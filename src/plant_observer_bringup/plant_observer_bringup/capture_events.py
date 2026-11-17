"""Local completion notifications; saved captures remain the source of truth."""
import hashlib
import os
import socket


def completion_address(capture_dir):
    # Linux abstract sockets need no files or stale-socket cleanup after a crash.
    path = os.fsencode(os.path.realpath(os.path.expanduser(capture_dir)))
    key = hashlib.sha256(path).hexdigest()[:32]
    return f'\0plant-captures-{os.getuid()}-{key}'


def completion_listener(capture_dir):
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        listener.bind(completion_address(capture_dir))
    except OSError:
        listener.close()
        raise
    return listener


def notify_completion(capture_dir):
    """Wake the uploader without waiting for Git or delaying the next patrol."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
        sender.settimeout(0.1)
        try:
            sender.sendto(b'finished', completion_address(capture_dir))
        except OSError:
            return False
    return True


def wait_for_completion(listener, retry_seconds):
    listener.settimeout(retry_seconds)
    try:
        listener.recv(32)
    except TimeoutError:
        return False
    return True
