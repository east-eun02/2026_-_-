"""Fallback provenance, fail-closed IPC, and real camera/model integration."""
import json
from pathlib import Path
import shutil
import sys
import time
from unittest.mock import Mock

import pytest

from plant_observer_bringup import change_compare as cc
from plant_observer_bringup import patrol_manager as pm
from plant_observer_bringup import review_server as rs
from plant_observer_bringup.model_fallback import ModelFallback, DEFAULT_PYTHON, DEFAULT_WEIGHTS
from test_previous_patrol import previous_node

ROUTE = Path(__file__).parents[2]/'plant_observer_gazebo/config/patrol_demo_real3.yaml'
CAPTURES = Path('/home/eun/plant_ws/captures_manual_demo_real3')


def records(tmp_path, before_lap='2026-10-31_004002', after_lap='2026-10-31_004554'):
    before, after = tmp_path/'2026-01-01_000000', tmp_path/'2026-01-01_000100'
    for folder, original_lap in ((before,before_lap),(after,after_lap)):
        folder.mkdir()
        shutil.copy2(CAPTURES/original_lap/'P00.png',folder/'P00.png')
        (folder/'result.json').write_text(json.dumps({'route':str(ROUTE),'finished':'2026-01-01T00:00:00',
            'stops':[{'id':'P00','status':'ok','image':'P00.png'}]}))
    return before,after


def invoke(root, current, fallback, **kwargs):
    return cc.compare_with_baseline(str(root),str(ROUTE),str(current),'P00',str(current/'P00.png'),
                                    model_fallback=fallback,**kwargs)


@pytest.mark.parametrize('status',['changed','unchanged','no_image'])
def test_valid_or_missing_image_primary_never_calls_model(tmp_path,monkeypatch,status):
    _,current=records(tmp_path)
    monkeypatch.setattr(cc,'compare_stop',Mock(return_value=({'status':status,'changes':{}},None)))
    fallback=Mock()
    result=invoke(tmp_path,current,fallback)
    assert result['status']==status and 'model_fallback' not in result
    fallback.compare.assert_not_called()


def test_missing_reference_never_calls_model(tmp_path):
    fallback=Mock()
    current=tmp_path/'2026-01-01_000100';current.mkdir()
    assert invoke(tmp_path,current,fallback)['status']=='no_reference'
    fallback.compare.assert_not_called()


def test_invalid_registration_never_bypassed(tmp_path,monkeypatch):
    _,current=records(tmp_path)
    monkeypatch.setattr(cc,'load_board_profile',Mock(side_effect=cc.BoardNotFound('invalid configuration')))
    fallback=Mock()
    assert invoke(tmp_path,current,fallback)['status']=='board_not_found'
    fallback.compare.assert_not_called()


def test_side_reference_not_recovered_by_front_model(tmp_path):
    _,current=records(tmp_path)
    fallback=Mock()
    result=invoke(tmp_path,current,fallback,view_id='left')
    assert result['status']=='no_reference'
    fallback.compare.assert_not_called()


@pytest.mark.parametrize('candidate_status',['changed','unchanged'])
def test_assisted_verdict_preserves_primary_and_requires_review(tmp_path,monkeypatch,candidate_status):
    _,current=records(tmp_path)
    primary={'status':'board_not_found','reason':'feature coverage'}
    monkeypatch.setattr(cc,'compare_stop',Mock(return_value=(primary,None)))
    fallback=Mock()
    fallback.compare.return_value=({'status':candidate_status,'changes':{},'comparison_scope':'model_bbox'},None,
                                   {'status':'used','model':{'name':'test'}})
    result=invoke(tmp_path,current,fallback)
    assert result['primary_result']==primary
    assert result['analysis_source']=='model_fallback' and result['requires_review']
    assert result['primary_status']=='board_not_found'
    assert result['reference_lap']=='2026-01-01_000000'
    assert rs.classify({'status':'ok','compare':result})[0]=='check'


@pytest.mark.parametrize('candidate', [None,{'status':'align_failed'}])
def test_failed_model_keeps_original_failure(tmp_path,monkeypatch,candidate):
    _,current=records(tmp_path)
    monkeypatch.setattr(cc,'compare_stop',Mock(return_value=({'status':'board_not_found','reason':'coverage'},None)))
    fallback=Mock()
    fallback.compare.return_value=(candidate,None,{'status':'failed','error':'timeout or alignment'})
    result=invoke(tmp_path,current,fallback)
    assert result['status']=='board_not_found' and result['reason']=='coverage'
    assert result['model_fallback']['status']=='failed' and 'requires_review' not in result


def test_missing_runtime_fails_without_importing_torch(tmp_path):
    fallback=ModelFallback(python=str(tmp_path/'no-python'),weights=str(tmp_path/'no-model'))
    result,picture,trace=fallback.compare('before','after',Mock(),Mock())
    assert result is picture is None and trace['status']=='failed'
    assert fallback.process is None


def test_partial_worker_output_times_out_and_process_is_reaped(tmp_path):
    worker=tmp_path/'hung.py'
    worker.write_text('import sys,time\nsys.stdin.readline()\nsys.stdout.write("{partial")\nsys.stdout.flush()\ntime.sleep(30)\n')
    weights=tmp_path/'weights';weights.write_bytes(b'test')
    fallback=ModelFallback(python=sys.executable,weights=str(weights),worker=str(worker),timeout=.3)
    start=time.monotonic()
    result,_,trace=fallback.compare('before','after',Mock(),Mock())
    assert result is None and 'TimeoutError' in trace['error']
    assert time.monotonic()-start < 5 and fallback.process is None


def test_model_unchanged_remains_pending_until_admin_review():
    result={'status':'unchanged','requires_review':True,'analysis_source':'model_fallback','changes':{}}
    stop={'status':'ok','final':result}
    assert pm.visit_priority(stop,None)[0]==2
    assert pm.visit_priority(stop,'확인 완료')[0]==4
    assert pm.visit_priority(stop,'오탐(변화 아님)')[0]==4
    assert not pm.needs_recheck('ok',result)


def test_patrol_final_keeps_provenance_and_changed_result_gets_sides(tmp_path):
    node,events,_=previous_node(tmp_path)
    pm.PatrolManager.run_lap(node,1)
    events.clear()
    node.compare_stop.side_effect=lambda folder,sid,image,**kw: {
        'status':'changed' if sid=='P00' else 'unchanged','changes':{'leaf_loss':1} if sid=='P00' else {},
        'analysis_source':'model_fallback','requires_review':True,'primary_status':'board_not_found'}
    result=pm.PatrolManager.run_lap(node,2)
    for stop in result['stops']:
        assert stop['final']['source']=='front'
        assert stop['final']['analysis_source']=='model_fallback' and stop['final']['requires_review']
    p00=next(s for s in result['stops'] if s['id']=='P00')
    p01=next(s for s in result['stops'] if s['id']=='P01')
    assert set(p00['views'])=={'front','left','right'} and set(p01['views'])=={'front'}
    observations=rs.Store(str(tmp_path)).observations()
    recent=[r for r in observations if r['lap']==Path(sorted(tmp_path.glob('*/result.json'))[-1]).parent.name]
    assert len(recent)==2 and all(r['kind']=='check' and '모델 보조' in r['text'] for r in recent)


@pytest.fixture(scope='module')
def real_model():
    if not (Path(DEFAULT_PYTHON).is_file() and Path(DEFAULT_WEIGHTS).is_file() and CAPTURES.exists()):
        pytest.skip('Local model runtime and real camera fixtures required')
    model=ModelFallback(timeout=30.)
    yield model
    model.close()


@pytest.mark.parametrize('before,after,expected',[('2026-10-31_004002','2026-10-31_004554','changed'),
                                                ('2026-10-31_003632','2026-10-31_004002','unchanged')])
def test_real_camera_real_model_through_public_api(tmp_path,real_model,before,after,expected):
    _,current=records(tmp_path,before,after)
    result=invoke(tmp_path,current,real_model)
    assert result['status']==expected,result
    assert result['analysis_source']=='model_fallback' and result['requires_review']
    assert result['primary_status']=='board_not_found'
    assert result['model_fallback']['model']['name']=='yolo11n-seg.pt'
    assert (current/result['image']).is_file()
    if expected=='changed':
        assert result['changes']=={'leaf_loss':1}
