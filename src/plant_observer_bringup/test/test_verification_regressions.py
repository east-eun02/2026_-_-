"""Regressions for startup failure, simulation geometry and upload recovery."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import threading
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from plant_observer_bringup import nav2_session as ns
from plant_observer_bringup import patrol_manager as pm
from plant_observer_bringup import change_compare as cc
from plant_observer_bringup import capture_uploader as cu
from test_system_verification import repos, lap, sync, git, LAP


def test_world_plants_match_route_targets():
    root=Path(get_package_share_directory('plant_observer_gazebo'))
    world=ET.parse(root/'worlds/demo.sdf')
    route=yaml.safe_load((root/'config/patrol_demo.yaml').read_text())
    # Map origin is Gazebo (0, -.4), facing north. Plant distance=.5m.
    for stop in route['stops']:
        pose=world.find(f"./world/model[@name='plant_{stop['id'][1:]}']/pose")
        x,y=map(float,pose.text.split()[:2])
        plant_map=(y+.4,-x)
        expected=(stop['x']+.5*math.cos(stop['yaw']),stop['y']+.5*math.sin(stop['yaw']))
        assert plant_map==pytest.approx(expected)


@pytest.mark.parametrize('matrix,valid',[
    (np.eye(3),True),
    (np.array([[0.,-1.,640.],[1.,0.,0.],[0.,0.,1.]]),True),
    (np.diag([1.8,.6,1.]),False),
    (np.array([[1.,1.7,0.],[0.,1.,0.],[0.,0.,1.]]),False),
    (np.diag([np.nan,1.,1.]),False)])
def test_alignment_rejects_stretch_even_with_unit_determinant(matrix,valid):
    assert cc.plausible(matrix)==valid


def test_startup_checks_localization_and_navigation_without_transition_requests(monkeypatch):
    node=SimpleNamespace(get_logger=lambda:Mock())
    clients={ns.service_name('',m,'is_active'):Mock() for m in ns.MANAGERS}
    calls=[]
    def call(n,t,name,request,**kw):
        calls.append(name)
        assert isinstance(request,ns.Trigger.Request)
        assert 0<kw['timeout']<=10
        return {'success':True}
    monkeypatch.setattr(ns,'call_service',call)
    assert ns.startup_stack(node,'',clients,10,threading.Event())['success']
    assert 'localization' in calls[0] and 'navigation' in calls[1]


def test_unready_localization_never_claims_navigation_ready(monkeypatch):
    node=SimpleNamespace(get_logger=lambda:Mock())
    clients={ns.service_name('',m,'is_active'):Mock() for m in ns.MANAGERS}
    call=Mock(return_value={'success':False,'reason':'response_timeout'})
    monkeypatch.setattr(ns,'call_service',call)
    clock=iter([0.,0.,0.,11.]);monkeypatch.setattr(ns.time,'monotonic',lambda:next(clock))
    monkeypatch.setattr(ns.rclpy,'spin_once',lambda *a,**kw:None)
    result=ns.startup_stack(node,'',clients,10,threading.Event())
    assert not result['success'] and call.call_count==1


def test_unavailable_manager_has_startup_deadline(monkeypatch):
    node=SimpleNamespace(get_logger=lambda:Mock())
    client=Mock();client.wait_for_service.return_value=False
    clients={ns.service_name('',m,'is_active'):client for m in ns.MANAGERS}
    clock=iter([0.,11.]);monkeypatch.setattr(ns.time,'monotonic',lambda:next(clock))
    assert ns.startup_stack(node,'',clients,10,threading.Event())['reason']=='startup_timeout'


def test_lost_readiness_response_can_be_retried_without_duplicate_startup(monkeypatch):
    node=SimpleNamespace(get_logger=lambda:Mock())
    clients={ns.service_name('',m,'is_active'):Mock() for m in ns.MANAGERS}
    call=Mock(side_effect=[{'success':False,'reason':'response_timeout'},
                          {'success':True},{'success':True}])
    monkeypatch.setattr(ns,'call_service',call)
    monkeypatch.setattr(ns.rclpy,'spin_once',lambda *a,**kw:None)
    result=ns.startup_stack(node,'',clients,10,threading.Event())
    assert result['success'] and [m['queries'] for m in result['managers']]==[2,1]
    assert all(isinstance(c.args[3],ns.Trigger.Request) for c in call.call_args_list)


def test_patrol_never_waits_forever_for_missing_nav2(monkeypatch):
    client=Mock();client.wait_for_service.return_value=False
    n=SimpleNamespace(startup_timeout=1.,create_client=lambda *a:client,destroy_client=Mock())
    clock=iter([0.,0.,2.]);monkeypatch.setattr(pm.time,'monotonic',lambda:next(clock))
    with pytest.raises(TimeoutError):pm.PatrolManager.wait_ready(n)
    n.destroy_client.assert_called_once_with(client)


def test_remote_update_is_rebased_without_losing_team_work(repos,tmp_path):
    captures,repo,remote=repos;lap(captures)
    teammate=tmp_path/'teammate'
    import subprocess
    subprocess.run(['git','clone',str(remote),str(teammate)],capture_output=True,check=True)
    git(teammate,'config','user.name','Teammate');git(teammate,'config','user.email','team@example.invalid')
    (teammate/'site.html').write_text('team update')
    git(teammate,'add','site.html');git(teammate,'commit','-m','site');git(teammate,'push')
    sync(captures,repo)
    assert git(remote,'cat-file','-e','main:site.html',check=False).returncode==0
    assert git(remote,'cat-file','-e',f'main:captures/{LAP}/P00.png',check=False).returncode==0


def test_pending_unrelated_commit_is_not_auto_pushed(repos):
    captures,repo,remote=repos;lap(captures)
    (repo/'draft.txt').write_text('do not publish')
    git(repo,'add','draft.txt');git(repo,'commit','-m','unrelated draft')
    with pytest.raises(cu.UploadError):sync(captures,repo)
    assert git(remote,'cat-file','-e','main:draft.txt',check=False).returncode!=0


def test_invalid_saved_index_is_rebuilt(repos):
    captures,repo,_=repos;lap(captures)
    (repo/'captures').mkdir();(repo/'captures/index.json').write_text('{incomplete')
    sync(captures,repo)
    assert json.loads((repo/'captures/index.json').read_text())['laps'][0]['lap']==LAP


def test_side_reference_image_path_and_final_verdict_are_preserved(repos):
    captures,repo,_=repos;folder,record=lap(captures)
    record['stops'][0]['views']['left']['compare']={
        'status':'changed','reference_lap':'2026-10-29_120000','reference_image':'P00_left.png',
        'image':'P00_left_compare.jpg','changes':{'leaf_color':2}}
    (folder/'result.json').write_text(json.dumps(record))
    cu.copy_lap(str(captures),str(repo/'captures'),LAP)
    result=cu.build_index(str(repo/'captures'))['laps'][0]['stops'][0]
    side=result['views']['left']
    assert side['image']==LAP+'/P00_left.png'
    assert side['compare']['reference_image']=='2026-10-29_120000/P00_left.png'
    assert result['final']['verdict']=='confirmed'


def test_duplicate_uploader_cannot_write_same_checkout(repos):
    import fcntl
    captures,repo,_=repos;lap(captures)
    with (repo/'.git/capture-uploader.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with pytest.raises(cu.UploadError):sync(captures,repo)
    assert not (repo/'captures').exists()


def test_changed_photo_is_repaired_even_if_result_is_identical(repos):
    captures,repo,_=repos;folder,_=lap(captures)
    sync(captures,repo)
    (repo/'captures'/LAP/'P00.png').write_bytes(b'truncated')
    assert sync(captures,repo)==[LAP]
    assert (repo/'captures'/LAP/'P00.png').read_bytes()==(folder/'P00.png').read_bytes()


def test_missing_manager_response_observes_stop_request():
    client=Mock();future=Mock();future.done.return_value=False
    client.wait_for_service.return_value=True;client.call_async.return_value=future
    stopping=threading.Event();stopping.set()
    node=SimpleNamespace(create_client=Mock(),destroy_client=Mock())
    result=ns.call_service(node,object,'/manager',object(),client=client,stopping=stopping)
    assert result['reason']=='stopped'
    future.cancel.assert_called_once()


def test_corrupt_review_file_is_preserved_when_save_fails(tmp_path):
    from plant_observer_bringup.review_server import Store
    file=tmp_path/'reviews.json';file.write_text('{bad')
    store=Store(str(tmp_path))
    with pytest.warns(UserWarning):assert store.observations()==[]
    with pytest.raises(ValueError):store.save_review('lap/P00','확인 완료','memo')
    assert file.read_text()=='{bad'


@pytest.mark.parametrize('status,changes,expected',[
    ('changed',{'leaf_color':2},'check'),('unchanged',{},'ok')])
def test_review_displays_successful_side_recovery_after_front_camera_failure(status,changes,expected):
    from plant_observer_bringup.review_server import classify
    stop={'status':'no_image','final':{'verdict':'recheck_used','status':status,'changes':changes}}
    kind,text=classify(stop)
    assert kind==expected and text!='사진 없음'
    if changes:assert '잎 색 변화 2곳' in text


def test_patrol_unanswered_state_query_times_out_and_destroys_client(monkeypatch):
    client=Mock();client.wait_for_service.return_value=True
    future=Mock();future.done.return_value=False;client.call_async.return_value=future
    n=SimpleNamespace(startup_timeout=1.,create_client=lambda *a:client,destroy_client=Mock())
    clock=iter([0.,0.,0.,2.,2.]);monkeypatch.setattr(pm.time,'monotonic',lambda:next(clock))
    monkeypatch.setattr(pm.rclpy,'spin_once',lambda *a,**kw:None)
    with pytest.raises(TimeoutError):pm.PatrolManager.wait_ready(n)
    future.cancel.assert_called_once();n.destroy_client.assert_called_once_with(client)


def test_stopped_simulation_clock_does_not_hang_capture_settle(monkeypatch):
    from rclpy.time import Time
    n=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=0)))
    clock=iter([0.,6.]);monkeypatch.setattr(pm.time,'monotonic',lambda:next(clock))
    with pytest.raises(TimeoutError):pm.PatrolManager.spin_for(n,1.)
