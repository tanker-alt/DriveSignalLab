#!/usr/bin/env python3
"""Python 3.6 process: execute original PressLight agent methods, JSON-line IPC."""
import contextlib,json,os,sys,random,pickle,ast,copy,tempfile
from pathlib import Path
from schema import PHASE_BITS
ROOT=Path(__file__).resolve().parents[4]
OFFICIAL=ROOT/'Drive/repos/PressLight_official'
sys.path.insert(0,str(OFFICIAL))
protocol=sys.stdout
with contextlib.redirect_stdout(sys.stderr):
    import numpy as np
    import tensorflow as tf
    from keras import backend as K
    from simple_dqn_agent import SimpleDQNAgent
    # Read only the official constant: importing config also imports CityFlow engine.
    tree=ast.parse((OFFICIAL/'config.py').read_text())
    DEFAULT=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='DIC_SIMPLEDQN_AGENT_CONF' for t in n.targets))
    K.set_session(tf.Session(config=tf.ConfigProto(intra_op_parallelism_threads=1,inter_op_parallelism_threads=1,device_count={'GPU':0})))
agent=None;memory=[];models=None;round_id=0
for line in sys.stdin:
    try:
        request=json.loads(line)
        with contextlib.redirect_stdout(sys.stderr):
            op=request['op']
            if op=='init':
                seed=request.get('seed',2026);random.seed(seed);np.random.seed(seed);tf.set_random_seed(seed)
                models=Path(request['models']);models.mkdir(parents=True,exist_ok=True)
                conf=copy.deepcopy(DEFAULT);conf.update(PRIORITY=False,UPDATE_Q_BAR_EVERY_C_ROUND=False)
                phases={i:bits[:] for i,bits in enumerate(PHASE_BITS)}
                env=dict(PHASE={'carla':phases},SIMULATOR_TYPE='carla',LANE_NUM={'LEFT':1,'STRAIGHT':1,'RIGHT':1},
                    LIST_STATE_FEATURE=['cur_phase','coming_vehicle','leaving_vehicle'],
                    DIC_FEATURE_DIM={'D_CUR_PHASE':(8,),'D_COMING_VEHICLE':(12,),'D_LEAVING_VEHICLE':(12,)},
                    BINARY_PHASE_EXPANSION=False,ONE_MODEL=False,TRANSFER=False)
                paths=dict(PATH_TO_MODEL=str(models),PATH_TO_WORK_DIRECTORY=str(models.parent))
                # Avoid upstream's CityFlow-specific automatic round_0_inter_0
                # filename when explicitly loading our CARLA checkpoint names.
                if any(models.iterdir()):
                    if not request.get('checkpoint'):raise ValueError('Nonempty models directory requires explicit checkpoint')
                    with tempfile.TemporaryDirectory(dir=str(models.parent)) as empty:
                        agent=SimpleDQNAgent(conf,env,dict(paths,PATH_TO_MODEL=empty),0)
                    agent.dic_path=paths
                else:agent=SimpleDQNAgent(conf,env,paths,0)
                if request.get('checkpoint'):agent.load_network(request['checkpoint'])
                if request.get('resume_replay'):
                    with (models.parent/'replay.pkl').open('rb') as f:memory=pickle.load(f)
                result={'ready':True,'agent_file':str(OFFICIAL/'simple_dqn_agent.py'),'config':conf,'env':env}
            elif op=='act':
                agent.dic_agent_conf['EPSILON']=request.get('epsilon',0.)
                result={'action':int(agent.choose_action(request.get('count',0),request['state']))}
            elif op=='train':
                memory.extend(request['samples']);memory=memory[-DEFAULT['MAX_MEMORY_LEN']:]
                round_id=request['round'];agent.cnt_round=round_id
                # Upstream loads max(round - lag, 0) on every round after 0.
                # Keeping the initial random target until round 5 diverges from it.
                if round_id>0:agent.load_network_bar('round_%03d'%max(round_id-conf['UPDATE_Q_BAR_FREQ'],0))
                if len(memory)>=4:
                    exp={'PRETRAIN':False,'AGGREGATE':False}
                    agent.prepare_Xs_Y(memory,exp);agent.train_network(exp)
                name='round_%03d'%round_id;agent.save_network(name)
                with (models.parent/'replay.pkl').open('wb') as f:pickle.dump(memory,f)
                result={'checkpoint':name,'memory':len(memory),'round':round_id}
            elif op=='load':
                agent.load_network(request['checkpoint']);result={'loaded':request['checkpoint']}
            elif op=='quit':
                break
            else:raise ValueError(op)
        protocol.write(json.dumps(result)+'\n');protocol.flush()
    except Exception:
        import traceback
        protocol.write(json.dumps({'error':traceback.format_exc()})+'\n');protocol.flush()
