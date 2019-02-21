import threading
from networktables import NetworkTables

SKYNET_IP = '10.44.16.2'
VISION_TABLE_NAME = "vision"
VISION_START_KEY = "captureProfiles"

LEFT_PROFILE_KEYS = ["leftProfile_0", "leftProfile_1", "leftProfile_2"]
RIGHT_PROFILE_KEYS = ["rightProfile_0", "rightProfile_1", "rightProfile_2"]

VISION_TABLE = None
profile_listener_cond_var = None
profile_listener_state = False

def require_vision(func):
    def _require_vision(*args, **kwargs):
        if VISION_TABLE is None:
            print("I'm not connected to the vision table. Please initialize it with init_and_wait().")
            return
        return func(*args, **kwargs)
    return _require_vision


def init_and_wait():
    """A blocking function that will init and wait"""
    global VISION_TABLE
    VISION_TABLE = NetworkTables.getTable(VISION_TABLE_NAME)
    cond = threading.Condition()
    notified = [False]

    def connectionListener(connected, info):
        print("Connected: {} \n info : {}".format(connected, info))
        with cond:
            notified[0] = True
            cond.notify()

    NetworkTables.initialize(server=SKYNET_IP)
    NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

    with cond:
        if not notified[0]:
            cond.wait()

@require_vision
def get_bool(key: str, default: bool) -> bool:
    return VISION_TABLE.getBoolean(key, default)

@require_vision
def set_bool(key: str, value: bool):
    VISION_TABLE.putBoolean(key, value)


def profile_request_listener(table, key, value, isNew):
    """A listener for waiting for the profile bool to be true"""
    if key != VISION_START_KEY:
        return

    with profile_listener_cond_var:
        global profile_listener_state
        profile_listener_state = value
        profile_listener_cond_var.notify()

@require_vision
def wait_for_profile_request():
    """Wait for a motion profile request from the main robot"""
    global profile_listener_cond_var
    global profile_listener_state

    profile_listener_cond_var = threading.Condition()
    profile_entry = NetworkTables.getEntry(VISION_START_KEY)
    profile_entry.addEntryListener(profile_request_listener)

    while profile_listener_cond_var:
        if not profile_listener_state:
            profile_listener_cond_var.wait()

    profile_listener_state = False

@require_vision
def send_motion_profiles(leftProfile, rightProfile):
    """Sends the motion profiles to the network table and turns off the required flag
    :param leftProfile, rightProfile: motion profiling profiles. Arrays sized 100x3"""
    leftEntries = [NetworkTables.getEntry(key) for key in LEFT_PROFILE_KEYS]
    rightEntries = [NetworkTables.getEntry(key) for key in RIGHT_PROFILE_KEYS]

    for i in range(3):
        leftEntries[i].putDoubleArray(leftProfile[i])
        rightEntries[i].putDoubleArray(rightProfile[i])

    set_bool(VISION_START_KEY, False)

if __name__ == "__main__":
    print("This is a library, so you can't run it as main.")
