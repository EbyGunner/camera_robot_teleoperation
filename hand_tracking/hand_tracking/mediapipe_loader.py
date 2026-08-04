"""Robust mediapipe import.

The original code hard-coded a relative path that resolved to
``<install>/hand_tracking/lib/src/...`` - a directory that does not exist
under the documented workspace layout - so the local install only worked
when mediapipe happened to be importable some other way.

Resolution order here:
  1. plain ``import mediapipe`` (global / venv install),
  2. ``$HAND_TRACKING_MEDIAPIPE_DIR`` if set,
  3. walk up from this file looking for a workspace ``src`` tree that
     contains ``hand_tracking/external_libraries`` (the location the
     README's ``pip3 install mediapipe --target=...`` step creates).
"""

import glob
import os
import sys


def _candidate_dirs():
    env = os.environ.get('HAND_TRACKING_MEDIAPIPE_DIR')
    if env:
        yield env

    here = os.path.abspath(os.path.dirname(__file__))
    parent = here
    for _ in range(8):                       # climb toward the workspace root
        parent = os.path.dirname(parent)
        for pattern in (
            os.path.join(parent, 'src', '*', 'hand_tracking',
                         'external_libraries'),
            os.path.join(parent, 'src', 'hand_tracking',
                         'external_libraries'),
            os.path.join(parent, 'hand_tracking', 'external_libraries'),
        ):
            yield from glob.glob(pattern)


def load_mediapipe(logger=None):
    """Import and return the mediapipe module, or raise ImportError with a
    message that explains all the supported install options."""
    try:
        import mediapipe  # noqa: F401
        return mediapipe
    except ImportError:
        pass

    for candidate in _candidate_dirs():
        if not os.path.isdir(candidate):
            continue
        if candidate not in sys.path:
            sys.path.insert(0, candidate)
        try:
            import mediapipe  # noqa: F401
            if logger:
                logger.info(f'mediapipe loaded from {candidate}')
            return mediapipe
        except ImportError:
            sys.path.remove(candidate)

    raise ImportError(
        'mediapipe not found. Either `pip3 install mediapipe`, or install '
        'it locally with `pip3 install mediapipe --target='
        '<ws>/src/camera_robot_teleoperation/hand_tracking/'
        'external_libraries`, or point HAND_TRACKING_MEDIAPIPE_DIR at the '
        'directory that contains the mediapipe package.')
