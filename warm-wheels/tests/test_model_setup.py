import hashlib
import io
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from pc.setup_models import fetch_checkpoint, verify_checkpoint


class ModelSetupTests(unittest.TestCase):
    def test_matching_local_checkpoint_avoids_network(self):
        content = b"known checkpoint bytes"
        source = {"sha256": hashlib.sha256(content).hexdigest(), "url": "https://example.invalid/model.pt"}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.pt"
            path.write_bytes(content)
            with patch("pc.setup_models.urllib.request.urlopen", side_effect=AssertionError("No network needed")):
                self.assertEqual(fetch_checkpoint(source, path), path)

    def test_changed_local_model_is_never_overwritten_or_loaded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.pt"
            path.write_bytes(b"unrecognized existing file")
            source = {"sha256": hashlib.sha256(b"expected").hexdigest(), "url": "https://example.invalid/model.pt"}
            with patch("pc.setup_models.urllib.request.urlopen", side_effect=AssertionError("Must not replace user file")):
                with self.assertRaisesRegex(RuntimeError, "does not match"):
                    fetch_checkpoint(source, path)
            self.assertEqual(path.read_bytes(), b"unrecognized existing file")

    def test_download_checksum_mismatch_leaves_no_checkpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.pt"
            source = {"sha256": hashlib.sha256(b"expected").hexdigest(), "url": "https://example.invalid/model.pt"}
            with patch("pc.setup_models.urllib.request.urlopen", return_value=io.BytesIO(b"wrong response")):
                with self.assertRaisesRegex(RuntimeError, "checksum"):
                    fetch_checkpoint(source, path)
            self.assertFalse(path.exists())
            self.assertFalse(path.with_suffix(".pt.download").exists())

    def test_unapproved_checkpoint_global_is_rejected_before_deserialization(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.pt"
            path.write_bytes(b"test")
            fake_torch = SimpleNamespace(serialization=SimpleNamespace(get_unsafe_globals_in_checkpoint=lambda _: ["os.system"]))
            with patch.dict("sys.modules", {"torch": fake_torch}):
                with self.assertRaisesRegex(RuntimeError, "unapproved"):
                    verify_checkpoint(path, hashlib.sha256(b"test").hexdigest())


if __name__ == "__main__":
    unittest.main()
