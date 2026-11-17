"""Read-only production audit: all writes go to pytest tempdirs/local bare remotes."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock
from urllib.request import Request, urlopen

import pytest

from plant_observer_bringup import capture_uploader as cu
from plant_observer_bringup import base_driver as bd
from plant_observer_bringup import review_server as rs


LAP='2026-10-30_210000'


def git(repo,*args,check=True):
    return subprocess.run(['git','-C',str(repo),*args],capture_output=True,text=True,check=check)


@pytest.fixture
def repos(tmp_path):
    remote=tmp_path/'remote.git';remote.mkdir()
    git(remote,'init','--bare','--initial-branch=main')
    repo=tmp_path/'checkout';repo.mkdir()
    git(repo,'init','--initial-branch=main')
    git(repo,'config','user.name','Local verification')
    git(repo,'config','user.email','test@example.invalid')
    (repo/'README.md').write_text('isolated audit fixture\n')
    git(repo,'add','README.md');git(repo,'commit','-m','fixture')
    git(repo,'remote','add','origin',str(remote));git(repo,'push','-u','origin','main')
    captures=tmp_path/'captures';captures.mkdir()
    return captures,repo,remote


def lap(captures):
    folder=captures/LAP;folder.mkdir()
    result={'route':'demo','started':'2026-10-30T21:00:00','finished':'2026-10-30T21:03:00',
            'home':'ok','stops':[{'id':'P00','status':'ok','image':'P00.png',
            'compare':{'status':'changed','change':'leaf_color','count':2,'changes':{'leaf_color':2}},
            'final':{'status':'changed','verdict':'confirmed','changes':{'leaf_color':2},
                     'agreeing_views':['front','left']},
            'views':{'front':{'status':'ok','image':'P00.png'},
                     'left':{'status':'ok','image':'P00_left.png'}}}]}
    (folder/'result.json').write_text(json.dumps(result))
    (folder/'P00.png').write_bytes(b'front photo fixture')
    (folder/'P00_left.png').write_bytes(b'left photo fixture')
    age=time.time()-20;os.utime(folder/'result.json',(age,age))
    return folder,result


def sync(captures,repo):
    return cu.sync(str(captures),str(repo),'captures',set())


def test_git_roundtrip_and_no_duplicate_commit(repos):
    captures,repo,remote=repos;folder,_=lap(captures)
    assert sync(captures,repo)==[LAP]
    head=git(repo,'rev-parse','HEAD').stdout
    assert head==git(remote,'rev-parse','main').stdout
    for name in ('result.json','P00.png','P00_left.png'):
        assert (repo/'captures'/LAP/name).read_bytes()==(folder/name).read_bytes()
    assert sync(captures,repo)==[]
    assert head==git(repo,'rev-parse','HEAD').stdout


def test_offline_upload_is_retried_without_losing_commit(repos,tmp_path):
    captures,repo,remote=repos;lap(captures)
    git(repo,'remote','set-url','origin',str(tmp_path/'unavailable.git'))
    with pytest.raises(cu.UploadError):
        sync(captures,repo)
    assert cu.ahead_count(str(repo))==1
    git(repo,'remote','set-url','origin',str(remote))
    sync(captures,repo)
    assert git(repo,'rev-parse','HEAD').stdout==git(remote,'rev-parse','main').stdout


def test_interrupted_copy_recovers_missing_photo_on_retry(repos,monkeypatch):
    captures,repo,remote=repos;lap(captures)
    original=cu.shutil.copy2
    def interrupted(src,dst):
        if Path(src).name=='P00_left.png':
            raise OSError('simulated copy interruption')
        return original(src,dst)
    monkeypatch.setattr(cu.shutil,'copy2',interrupted)
    with pytest.raises(OSError):sync(captures,repo)
    assert (repo/'captures'/LAP/'result.json').exists()
    monkeypatch.setattr(cu.shutil,'copy2',original)
    sync(captures,repo)
    assert (repo/'captures'/LAP/'P00_left.png').exists(), 'same result.json must not hide a missing photo'
    assert git(remote,'cat-file','-e',f'main:captures/{LAP}/P00_left.png',check=False).returncode==0


def test_index_keeps_final_verdict_and_side_views(repos):
    captures,repo,_=repos;_,record=lap(captures)
    sync(captures,repo)
    stop=json.loads((repo/'captures/index.json').read_text())['laps'][0]['stops'][0]
    assert stop.get('final')==record['stops'][0]['final'], 'GitHub summary drops confirmed/mismatch and agreeing_views'
    assert stop.get('views'), 'GitHub summary must expose the captured side views'


def test_upload_does_not_publish_unrelated_staged_files(repos):
    captures,repo,remote=repos;lap(captures)
    (repo/'unrelated-draft.txt').write_text('not selected for publishing\n')
    git(repo,'add','unrelated-draft.txt')
    sync(captures,repo)
    assert git(remote,'cat-file','-e','main:unrelated-draft.txt',check=False).returncode!=0
    assert git(repo,'diff','--cached','--name-only').stdout.strip()=='unrelated-draft.txt'


def test_once_reports_push_failure_as_nonzero_exit(repos,tmp_path):
    captures,repo,_=repos;lap(captures)
    git(repo,'remote','set-url','origin',str(tmp_path/'unavailable.git'))
    result=subprocess.run([sys.executable,'-m','plant_observer_bringup.capture_uploader',
        '--capture-dir',str(captures),'--repo',str(repo),'--once'],capture_output=True,text=True,timeout=15)
    assert result.returncode!=0, f'Unpublished commit reported exit 0: {result.stdout}'


@pytest.fixture
def review_http(tmp_path):
    captures=tmp_path/'review';captures.mkdir();lap(captures)
    server=rs.ThreadingHTTPServer(('127.0.0.1',0),rs.make_handler(rs.Store(str(captures))))
    worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
    yield captures,f'http://127.0.0.1:{server.server_port}'
    server.shutdown();server.server_close();worker.join(timeout=2)


def test_review_http_roundtrip_and_side_photos(review_http):
    captures,url=review_http
    with urlopen(url+'/api/observations',timeout=2) as response:
        item=json.load(response)['items'][0]
    assert item['verdict']=='confirmed'
    assert item['side_images'][0]['image'].endswith('P00_left.png')
    body=json.dumps({'key':LAP+'/P00','result':'재촬영 필요','memo':'audit only'}).encode()
    with urlopen(Request(url+'/api/review',data=body,headers={'Content-Type':'application/json'}),timeout=2) as response:
        assert json.load(response)['result']=='재촬영 필요'
    assert json.loads((captures/'reviews.json').read_text())[LAP+'/P00']['memo']=='audit only'
    with urlopen(url+'/api/export.csv',timeout=2) as response:
        assert '재촬영 필요' in response.read().decode()


def test_one_corrupt_lap_does_not_hide_all_review_records(review_http):
    captures,url=review_http
    corrupt=captures/'2026-10-30_200000';corrupt.mkdir()
    (corrupt/'result.json').write_text('{broken')
    with urlopen(url+'/api/observations',timeout=2) as response:
        assert json.load(response)['items'][0]['lap']==LAP


def test_wheel_speed_limit_preserves_curvature():
    left,right=bd.wheel_rpms(.5,1.,.033,.287,30.)
    assert max(abs(left),abs(right))==pytest.approx(30.)
    assert left/right==pytest.approx((.5-.287/2)/(.5+.287/2))


def test_wheel_turn_and_signed_encoder():
    left,right=bd.wheel_rpms(0.,.5,.033,.287,30.)
    assert left<0<right and abs(left)==pytest.approx(right)
    assert bd.to_signed32(0xffffffff)==-1
    assert bd.to_signed32(0x80000000)==-2147483648


class ClockValue:
    def __init__(self,nanoseconds):self.nanoseconds=nanoseconds
    def __sub__(self,other):return ClockValue(self.nanoseconds-other.nanoseconds)


@pytest.mark.parametrize('age,stopped',[(0.1,False),(.6,True)])
def test_command_timeout_stops_wheel_outputs(age,stopped):
    writer=Mock();writer.txPacket.return_value=bd.COMM_SUCCESS
    n=SimpleNamespace(cmd=(.1,0.),last_cmd_time=ClockValue(0),cmd_timeout=.5,
        wheel_radius=.033,wheel_separation=.287,max_wheel_rpm=30.,sync_write=writer,
        left_id=1,right_id=2,left_sign=1,right_sign=-1)
    assert bd.BaseDriver.send_velocity(n,ClockValue(int(age*1e9)))
    commands=[int.from_bytes(bytes(call.args[1]),'little',signed=True) for call in writer.addParam.call_args_list]
    assert (commands==[0,0])==stopped
    if not stopped:assert commands[0]>0 and commands[1]<0


def test_communication_failure_latches_fault_and_stops_motors():
    n=SimpleNamespace(fault=False,comm_failures=4,max_comm_failures=5,
        get_clock=lambda:SimpleNamespace(now=lambda:ClockValue(0)),
        read_ticks=lambda:None,send_velocity=lambda now:False,
        stop_motors=Mock(),get_logger=lambda:Mock())
    n.enter_fault=lambda reason:bd.BaseDriver.enter_fault(n,reason)
    bd.BaseDriver.update(n)
    assert n.fault
    n.stop_motors.assert_called_once()
