"""Shutdown ordering, bounded waits, and ownership of process signals."""
from types import SimpleNamespace
from unittest.mock import Mock
import signal
import subprocess

import pytest

from plant_observer_bringup import nav2_session as ns


def node():
    return SimpleNamespace(get_logger=lambda: Mock(), create_client=Mock(), destroy_client=Mock())


def test_shutdown_navigation_before_localization(monkeypatch):
    calls=[]
    def call(n,typ,name,request,*a,**kw):
        calls.append((name,request))
        return {'service':name,'success':True}
    monkeypatch.setattr(ns,'call_service',call)
    result=ns.shutdown_stack(node(),'/robot_a')
    assert result['success']
    transitions=[(name,request) for name,request in calls if name.endswith('/manage_nodes')]
    assert [x[0] for x in transitions]==[
        '/robot_a/lifecycle_manager_navigation/manage_nodes',
        '/robot_a/lifecycle_manager_localization/manage_nodes']
    assert all(request.command==request.SHUTDOWN for _,request in transitions)
    assert all(name.startswith('/robot_a/') for name,_ in calls)


def test_failed_navigation_shutdown_preserves_localization(monkeypatch):
    calls=[]
    def call(n,typ,name,request,*a,**kw):
        calls.append(name)
        return {'service':name,'success':False,'reason':'response_timeout'}
    monkeypatch.setattr(ns,'call_service',call)
    assert not ns.shutdown_stack(node())['success']
    assert not any('localization' in name for name in calls)


def test_missing_service_is_explicit_failure_and_client_is_destroyed():
    n=node();client=Mock();client.wait_for_service.return_value=False
    n.create_client=Mock(return_value=client);n.destroy_client=Mock()
    result=ns.call_service(n,object,'/missing',object(),discovery_timeout=.1)
    assert result['reason']=='service_unavailable'
    n.destroy_client.assert_called_once_with(client)
    client.call_async.assert_not_called()


def test_unanswered_service_has_wall_time_deadline(monkeypatch):
    n=node();client=Mock();future=Mock()
    client.wait_for_service.return_value=True
    client.call_async.return_value=future;future.done.return_value=False
    n.create_client=Mock(return_value=client);n.destroy_client=Mock()
    clock=iter([0.,0.,2.])
    monkeypatch.setattr(ns.time,'monotonic',lambda:next(clock))
    result=ns.call_service(n,object,'/slow',object(),timeout=1.)
    assert result['reason']=='response_timeout'
    future.cancel.assert_called_once()
    n.destroy_client.assert_called_once_with(client)


def test_rejected_lifecycle_transition_is_not_success(monkeypatch):
    n=node();client=Mock();future=Mock()
    client.wait_for_service.return_value=True;client.call_async.return_value=future
    future.done.return_value=True;future.result.return_value=SimpleNamespace(success=False)
    n.create_client=Mock(return_value=client);n.destroy_client=Mock()
    assert not ns.call_service(n,object,'/manager',object())['success']


def test_persistent_client_is_not_destroyed_after_request():
    n=node();client=Mock();future=Mock()
    client.wait_for_service.return_value=True;client.call_async.return_value=future
    future.done.return_value=True;future.result.return_value=SimpleNamespace(success=True)
    assert ns.call_service(n,object,'/manager',object(),client=client)['success']
    n.create_client.assert_not_called()
    n.destroy_client.assert_not_called()


def test_all_shutdown_endpoints_exist_before_first_request(monkeypatch):
    n=node();n.create_client.side_effect=lambda *a: Mock()
    def call(*a,**kw):
        assert n.create_client.call_count==10
        assert n.destroy_client.call_count==0
        assert kw['client'] is not None
        return {'success':True}
    monkeypatch.setattr(ns,'call_service',call)
    assert ns.shutdown_stack(n)['success']
    assert n.destroy_client.call_count==10


def test_cancel_waits_for_terminal_result_before_lifecycle_shutdown(monkeypatch):
    calls=[]
    uuid=list(range(16))
    def call(n,typ,name,request,*a,**kw):
        calls.append(name)
        if name=='/navigate_to_pose/_action/cancel_goal':
            return {'success':True,'goals_canceling':[uuid]}
        if name.endswith('/get_result'):
            assert list(request.goal_id.uuid)==uuid
        return {'success':True}
    monkeypatch.setattr(ns,'call_service',call)
    assert ns.shutdown_stack(node())['success']
    assert calls.index('/navigate_to_pose/_action/get_result') < calls.index(
        '/lifecycle_manager_navigation/manage_nodes')


@pytest.mark.parametrize('status,expected', [(0,False),(4,True),(5,True),(6,True)])
def test_result_requires_terminal_action_state(status,expected):
    n=node();client=Mock();future=Mock()
    client.wait_for_service.return_value=True;client.call_async.return_value=future
    future.done.return_value=True;future.result.return_value=SimpleNamespace(status=status)
    assert ns.call_service(n,object,'/result',object(),client=client)['success']==expected


def test_ros_goal_uuid_is_json_serializable():
    import json
    from action_msgs.msg import GoalInfo
    response=ns.CancelGoal.Response()
    response.goals_canceling=[GoalInfo()]
    n=node();client=Mock();future=Mock()
    client.wait_for_service.return_value=True;client.call_async.return_value=future
    future.done.return_value=True;future.result.return_value=response
    result=ns.call_service(n,object,'/cancel',object(),client=client)
    assert json.loads(json.dumps(result))['goals_canceling']==[[0]*16]


def test_failed_startup_skips_invalid_lifecycle_transitions(monkeypatch):
    calls=[]
    def call(n,typ,name,*a,**kw):
        calls.append(name)
        return {'service':name,'success':not name.endswith('/is_active')}
    monkeypatch.setattr(ns,'call_service',call)
    result=ns.shutdown_stack(node(),stop_process=True)
    assert result['success']
    assert all(r['reason']=='inactive_process_cleanup' for r in result['managers'])
    assert not any(name.endswith('/manage_nodes') for name in calls)


def test_standalone_helper_does_not_claim_to_stop_inactive_processes(monkeypatch):
    monkeypatch.setattr(ns,'call_service',lambda *a,**kw:{'success':False})
    result=ns.shutdown_stack(node())
    assert not result['success']
    assert result['managers'][0]['reason']=='inactive_process_cleanup'


def test_exited_process_never_receives_more_signals(monkeypatch):
    proc=Mock();proc.poll.return_value=0;proc.returncode=0
    kill=Mock();monkeypatch.setattr(ns.os,'killpg',kill)
    result=ns.stop_launch(proc)
    assert result=={'exit_code':0,'signals':[],'forced':False}
    proc.send_signal.assert_not_called();kill.assert_not_called()


def test_successful_sigint_never_escalates(monkeypatch):
    proc=Mock();proc.poll.return_value=None;proc.returncode=0
    kill=Mock();monkeypatch.setattr(ns.os,'killpg',kill)
    result=ns.stop_launch(proc)
    proc.send_signal.assert_called_once_with(signal.SIGINT)
    assert not result['forced']
    kill.assert_not_called()


def test_timeout_escalates_only_owned_live_group(monkeypatch):
    proc=Mock();proc.pid=12345;proc.poll.return_value=None;proc.returncode=-15
    proc.wait.side_effect=[subprocess.TimeoutExpired('owned',12),-15]
    kill=Mock();monkeypatch.setattr(ns.os,'killpg',kill)
    result=ns.stop_launch(proc)
    assert result['forced']
    kill.assert_called_once_with(12345,signal.SIGTERM)


def test_exit_during_timeout_does_not_signal_reused_pid(monkeypatch):
    proc=Mock();proc.poll.side_effect=[None,0];proc.returncode=0
    proc.wait.side_effect=subprocess.TimeoutExpired('owned',12)
    kill=Mock();monkeypatch.setattr(ns.os,'killpg',kill)
    result=ns.stop_launch(proc)
    assert not result['forced']
    kill.assert_not_called()


def test_report_preserves_unclean_shutdown_details(tmp_path):
    import json
    path=tmp_path/'nested'/'shutdown.json'
    data={'clean':False,'shutdown_errors':['crashed'],'process':{'forced':True}}
    ns.write_report(path,data)
    assert json.loads(path.read_text())==data
    assert not path.with_name(path.name+'.tmp').exists()
