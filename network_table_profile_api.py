import threading
from networktables import NetworkTables

SKYNET_IP = '10.44.16.2'
VISION_TABLE_NAME = "vision"
VISION_START_KEY = "captureProfiles"

LEFT_PROFILE_KEYS = ["leftProfile_0", "leftProfile_1", "leftProfile_2"]
RIGHT_PROFILE_KEYS = ["rightProfile_0", "rightProfile_1", "rightProfile_2"]

vision_table = None
profile_listener_cond_var = None
profile_listener_state = False


def init_and_wait():
    """
    A blocking function that will init and wait
    """
    global vision_table
    vision_table = NetworkTables.getTable(VISION_TABLE_NAME)
    cond = threading.Condition()
    notified = [False]

    def connection_listener(connected, info):
        print("Connected: {} \n info : {}".format(connected, info))
        with cond:
            notified[0] = True
            cond.notify()

    NetworkTables.initialize(server=SKYNET_IP)
    NetworkTables.addConnectionListener(connection_listener,
                                        immediateNotify=True)

    with cond:
        if not notified[0]:
            cond.wait()


def get_bool(key: str, default: bool) -> bool:
    """
    Gets a boolean value from the table if initiated
    """
    if vision_table is None:
        print("vision_table wasn't initialized, please use init")
        return
    return vision_table.getBoolean(key, default)


def set_bool(key: str, value: bool):
    """
    Sets a boolean value in the table if initiated
    """
    if vision_table is None:
        print("vision_table wasn't initialized, please use init")
        return
    vision_table.putBoolean(key, value)


def profile_request_listener(table, key, value, is_new):
    """
    A listener for waiting for the profile bool to be true
    """
    if key != VISION_START_KEY:
        return

    with profile_listener_cond_var:
        global profile_listener_state
        profile_listener_state = value
        profile_listener_cond_var.notify()


def wait_for_profile_request():
    """
    Waits for a motion profile request from the main robot
    """
    if vision_table is None:
        raise Exception("vision_table wasn't initialized, please use init")

    global profile_listener_cond_var
    global profile_listener_state

    profile_listener_cond_var = threading.Condition()
    profile_entry = NetworkTables.getEntry(VISION_START_KEY)
    profile_entry.addEntryListener(profile_request_listener)

    while profile_listener_cond_var:
        if not profile_listener_state:
            profile_listener_cond_var.wait()

    profile_listener_state = False


def send_motion_profiles(left_profile, right_profile):
    """
    Sends the motion profiles to the network table
    and turns off the required flag
    :param left_profile: motion profiling profile. Array sized 100x3
    :param right_profile: same as left profile
    """
    if vision_table is None:
        raise Exception("vision_table wasn't initialized, please use init")

    left_entries = [NetworkTables.getEntry(key) for key in LEFT_PROFILE_KEYS]
    right_entries = [NetworkTables.getEntry(key) for key in RIGHT_PROFILE_KEYS]

    for i in range(3):
        left_entries[i].putDoubleArray(left_profile[i])
        right_entries[i].putDoubleArray(right_profile[i])

    set_bool(VISION_START_KEY, False)


if __name__ == "__main__":
    print("This is a library, so you can't run it as main.")
