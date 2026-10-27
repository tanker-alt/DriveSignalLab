import unittest
from pathlib import Path
from unittest.mock import patch
import runtime_config


class WorkspaceTests(unittest.TestCase):
    def test_default_and_containment(self):
        root = runtime_config.workspace_root()
        self.assertEqual(runtime_config.map_output(root/'maps'/'example'), root/'maps'/'example')
        with self.assertRaises(ValueError):
            runtime_config.map_output(root.parent/'outside-map')
        with self.assertRaises(ValueError):
            runtime_config.map_output(root/'maps'/'..'/'..'/'outside-map')

    def test_explicit_local_interpreter(self):
        import sys
        with patch.object(runtime_config, 'settings', return_value={'generator_python': sys.executable}):
            self.assertEqual(Path(runtime_config.python_for('generator')), Path(sys.executable).resolve())


if __name__ == '__main__':
    unittest.main()
