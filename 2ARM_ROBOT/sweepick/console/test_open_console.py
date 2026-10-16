from types import SimpleNamespace
import unittest
from unittest.mock import patch

import open_console


class ConsoleEntry(unittest.TestCase):
    def test_existing_server_opens_without_launching_another_process(self):
        with patch.object(open_console, 'console_ready', return_value=True), patch.object(open_console.subprocess, 'run') as run:
            open_console.ensure_console()
        run.assert_not_called()

    def test_missing_server_starts_only_console_once(self):
        with patch.object(open_console, 'console_ready', side_effect=[False, False, True]), \
                patch.object(open_console.shutil, 'which', return_value='/usr/bin/tmux'), \
                patch.object(open_console.time, 'sleep'), \
                patch.object(open_console.subprocess, 'run', side_effect=[SimpleNamespace(returncode=1), SimpleNamespace(returncode=0)]) as run:
            open_console.ensure_console()
        self.assertEqual(len(run.call_args_list), 2)
        command = run.call_args_list[1].args[0]
        self.assertEqual(command[:5], ['tmux', 'new-session', '-d', '-s', 'sweepick-console'])
        self.assertIn('app.py', command[-1])
        self.assertNotIn('RUN_APPROVED_SESSION', command[-1])

    def test_existing_starting_console_is_not_duplicated(self):
        with patch.object(open_console, 'console_ready', side_effect=[False, True]), \
                patch.object(open_console.shutil, 'which', return_value='/usr/bin/tmux'), \
                patch.object(open_console.subprocess, 'run', return_value=SimpleNamespace(returncode=0)) as run:
            open_console.ensure_console()
        run.assert_called_once_with(['tmux', 'has-session', '-t', 'sweepick-console'], capture_output=True)

    def test_unrelated_port_is_not_overwritten(self):
        with patch.object(open_console, 'console_ready', side_effect=RuntimeError('foreign server')), \
                patch.object(open_console.subprocess, 'run') as run:
            with self.assertRaisesRegex(RuntimeError, 'foreign server'):
                open_console.ensure_console()
        run.assert_not_called()

    def test_the_entry_opens_the_main_dashboard_not_the_removed_instructor_page(self):
        text = open_console.Path(open_console.__file__).read_text()
        self.assertNotIn('/instructor', text)
        with patch.object(open_console, 'ensure_console'), patch.object(open_console.webbrowser, 'open', return_value=True) as opened, \
                patch('sys.argv', ['open_console.py']):
            self.assertEqual(open_console.main(), 0)
        opened.assert_called_once_with(open_console.URL + '/')


if __name__ == '__main__':
    unittest.main()
