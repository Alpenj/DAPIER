from pathlib import Path
import tempfile
import unittest

from shoe_sorting_data.perception_exemplar import (
    add_perception_exemplar,
    build_perception_registry,
    load_perception_registry,
    match_perception_exemplar,
    save_perception_registry,
)


class PerceptionExemplarTest(unittest.TestCase):
    def make_registry(self):
        registry = build_perception_registry("synthetic_embedding_v1")
        registry = add_perception_exemplar(
            registry,
            exemplar_id="shoe_a_left",
            pair_id="pair_a",
            object_instance_id="shoe_a_left_object",
            embedding=[1.0, 0.0, 0.0],
            session_id="session_train_a",
            background_id="background_train",
            features={"color": "black"},
        )
        return add_perception_exemplar(
            registry,
            exemplar_id="shoe_b_left",
            pair_id="pair_b",
            object_instance_id="shoe_b_left_object",
            embedding=[0.0, 1.0, 0.0],
            session_id="session_train_b",
            background_id="background_train",
            features={"color": "white"},
        )

    def test_match_and_round_trip_never_authorize_control(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "pair_registry.json"
            save_perception_registry(path, self.make_registry())
            result = match_perception_exemplar(load_perception_registry(path), [0.99, 0.01, 0.0])
            self.assertEqual(result["decision"], "match")
            self.assertEqual(result["pair_id"], "pair_a")
            self.assertFalse(result["control_authorized"])

    def test_ambiguous_query_abstains(self):
        result = match_perception_exemplar(
            self.make_registry(),
            [1.0, 1.0, 0.0],
            min_similarity=0.5,
            min_margin=0.1,
        )
        self.assertEqual(result["decision"], "abstain")
        self.assertEqual(result["reason"], "ambiguous_margin")
        self.assertIsNone(result["pair_id"])

    def test_dimension_mismatch_and_duplicate_id_are_rejected(self):
        registry = self.make_registry()
        with self.assertRaisesRegex(ValueError, "expected 3"):
            match_perception_exemplar(registry, [1.0, 0.0])
        with self.assertRaisesRegex(ValueError, "duplicate exemplar_id"):
            add_perception_exemplar(
                registry,
                exemplar_id="shoe_a_left",
                pair_id="pair_c",
                object_instance_id="shoe_c_left_object",
                embedding=[0.0, 0.0, 1.0],
                session_id="session_train_c",
                background_id="background_train",
            )


if __name__ == "__main__":
    unittest.main()
