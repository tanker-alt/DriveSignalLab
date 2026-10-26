"""Exercise official updates, target reload and HDF5 action round-trip (Python 3.6)."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import h5py
import numpy as np
from schema import PHASE_BITS
from learning import IntervalReturn

HERE=Path(__file__).resolve().parent


class WorkerTest(unittest.TestCase):
    def test_train_target_and_reload(self):
        with tempfile.TemporaryDirectory(prefix='presslight_contract_') as tmp:
            root=Path(tmp);models=root/'models'
            env=dict(os.environ,CUDA_VISIBLE_DEVICES='-1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',TF_CPP_MIN_LOG_LEVEL='2')
            def start(log):
                return subprocess.Popen([sys.executable,'-u',str(HERE/'worker.py')],stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,stderr=log,universal_newlines=True,env=env)
            def call(process,**request):
                process.stdin.write(json.dumps(request)+'\n');process.stdin.flush()
                response=json.loads(process.stdout.readline())
                self.assertNotIn('error',response,response)
                return response
            def stop(process):
                if process.poll() is None:
                    process.stdin.write('{"op":"quit"}\n');process.stdin.flush();process.wait(timeout=30)
                process.stdin.close();process.stdout.close()
            samples=[]
            for i in range(12):
                state=dict(cur_phase=PHASE_BITS[i%4],coming_vehicle=[int(j==i) for j in range(36)],leaving_vehicle=[0]*36)
                following=dict(state,cur_phase=PHASE_BITS[(i+1)%4])
                interval=IntervalReturn();interval.add(-.25*(i%3),10+i)
                samples.append(interval.sample(state,i%4,following,i*10,i%3==0,False))
            with (root/'worker.log').open('w') as log:
                process=start(log)
                try:
                    config=call(process,op='init',models=str(models),seed=2026,warmup=4)
                    self.assertTrue(config['agent_file'].endswith('PressLight_official/simple_dqn_agent.py'))
                    self.assertEqual(call(process,op='train',samples=samples,round=0)['memory'],12)
                    self.assertEqual(call(process,op='train',samples=samples,round=1)['memory'],24)
                    actions=[call(process,op='act',state=s['state'],epsilon=0)['action'] for s in samples]
                finally:stop(process)
            self.assertIn('succeed in loading model round_000',(root/'worker.log').read_text())
            with h5py.File(str(models/'round_001.h5'),'r') as checkpoint:
                weights=[]
                for name in ('dense_0','dense_1','q_values'):
                    layer=checkpoint['model_weights'][name][name]
                    weights.append((np.array(layer['kernel:0']),np.array(layer['bias:0'])))
            self.assertEqual(weights[0][0].shape,(80,20))
            self.assertEqual(weights[-1][1].shape,(4,))
            for sample,action in zip(samples,actions):
                state=sample['state'];value=np.array(state['cur_phase']+state['coming_vehicle']+state['leaving_vehicle'],dtype='float32')
                for i,(weight,bias) in enumerate(weights):
                    value=np.dot(value,weight)+bias
                    if i<2:value=np.maximum(value,0)
                self.assertEqual(action,int(value.argmax()))
            with (root/'reload.log').open('w') as log:
                process=start(log)
                try:
                    call(process,op='init',models=str(models),seed=2026,checkpoint='round_001',resume_replay=False)
                    reloaded=[call(process,op='act',state=s['state'],epsilon=0)['action'] for s in samples]
                    self.assertEqual(actions,reloaded)
                finally:stop(process)


if __name__=='__main__':unittest.main()
