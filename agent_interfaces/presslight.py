"""Official PressLight inference in its existing isolated Python runtime."""
import json
from pathlib import Path
import subprocess

from .signals import SignalAgent


class OfficialPressLightAgent(SignalAgent):
    name = 'official_presslight'

    def __init__(self, python, models, checkpoint, log, seed=2026):
        worker = Path(__file__).resolve().parents[1]/'integrations/cross3/official/worker.py'
        self.log = Path(log).open('a')
        self.process = None
        try:
            self.process = subprocess.Popen([python, '-u', str(worker)], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, stderr=self.log,
                                            universal_newlines=True)
            self._call(op='init', models=str(Path(models).resolve()), checkpoint=checkpoint, seed=seed)
        except Exception:
            self.close()
            raise

    def _call(self, **request):
        self.process.stdin.write(json.dumps(request)+'\n')
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError('PressLight worker exited; inspect the configured log')
        response = json.loads(line)
        if 'error' in response:
            raise RuntimeError(response['error'])
        return response

    def reset(self, context):
        if context['observation_schema'] != 'cross3_official_80_v1':
            raise ValueError('Official PressLight requires the official 80-D environment; counts cannot substitute')

    def act(self, observation):
        return self._call(op='act', state=observation['raw']['state'], epsilon=0.)['action']

    def close(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
            self.process.stdin.close()
            self.process.stdout.close()
        self.log.close()
