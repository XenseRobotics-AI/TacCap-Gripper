"""UI-only payloads: never depend on untracked vendor motor firmware."""

import hashlib
from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch


@contextmanager
def rs00_ui_image(window):
    """Keep real image validation/default selection with a test-only catalog."""
    import bundled

    original_catalog = window.catalog
    catalog = deepcopy(original_catalog)
    relative = bundled.FILES["motor", "RS00"]
    entry = next(e for e in catalog["images"] if e.get("file") == relative)
    data = b"Synthetic RS00 UI fixture; NOT motor firmware.\n"
    entry["sha256"] = hashlib.sha256(data).hexdigest()
    default_image = bundled.default_image

    with TemporaryDirectory() as folder:
        fixture_root = Path(folder)
        path = fixture_root / relative
        path.parent.mkdir(parents=True)
        path.write_bytes(data)

        def select(kind, model, selected_catalog, root=None):
            if (kind, model) == ("motor", "RS00"):
                root = fixture_root
            return default_image(kind, model, selected_catalog, root)

        window.catalog = catalog
        try:
            with patch.object(bundled, "default_image", side_effect=select):
                yield path
        finally:
            window.catalog = original_catalog
