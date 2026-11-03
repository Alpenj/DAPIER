from contextlib import redirect_stderr, redirect_stdout
import io
import json
import unittest

from casino_dealer.cli import build_parser, main


class CasinoDealerCliTest(unittest.TestCase):

    def test_compact_plan_is_valid_json(self):
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            exit_code = main(['--players', '3', '--compact'])

        plan = json.loads(stdout.getvalue())
        self.assertEqual(exit_code, 0)
        self.assertEqual(plan['game']['player_count'], 3)
        self.assertEqual(len(plan['commands']), 18)

    def test_invalid_player_count_is_a_cli_usage_error(self):
        stderr = io.StringIO()
        with redirect_stderr(stderr):
            with self.assertRaisesRegex(SystemExit, '2'):
                build_parser().parse_args(['--players', '0'])

        self.assertIn('players must be between 1 and 7', stderr.getvalue())


if __name__ == '__main__':
    unittest.main()
