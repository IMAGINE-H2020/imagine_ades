#!/usr/bin/env python

import rospy
import numpy as np
from imagine_common.srv import ListAdes, GetAdesPreConds, GetAdesEffects, GetAdesMotions, GetAdesMotionNames, StoreAdes, UpdateAdes, DeleteAdes, ListMotion, GetMotion, StoreMotion, UpdateMotion, DeleteMotion
from imagine_common.msg import Motion, KeyValPair, AdesData, MotionSequence
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension, String

if __name__ == "__main__":
    print("Converting ADES to ADES2 ...")
    print("This process requires to run in parallel an adesdb (1) and ades2db+motiondb (2)")
    print("The content of (1) will be copied and adapted to (2)")
    print("Be sure NOT to override data in the process.")

    raw_input("Continue (to abort: Ctrl+C)")

    print("banana")
    exit()


    print("Don't use your original databases to avoid data loss/corruption")
    print("Start a dummy ADES2 database")
    rospy.wait_for_service("/ades2db/list_ades")
    print("list_ades available, ADES2DB is running.")
    print("Start a dummy motion database")
    rospy.wait_for_service("/motiondb/list_motion")
    print("list_motion available, MotionDB is running.")

    # Service proxies
    print("Testing MotionDB")
    list_motions = rospy.ServiceProxy("motiondb/list_motion", ListMotion)
    delete_motion = rospy.ServiceProxy("motiondb/delete_motion", DeleteMotion)
    store_motion = rospy.ServiceProxy("motiondb/store_motion", StoreMotion)
    update_motion = rospy.ServiceProxy("motiondb/update_motion", UpdateMotion)
    get_motion = rospy.ServiceProxy("motiondb/get_motion", GetMotion)

    # Testing whether the DB is empty, if not raising a warning
    motions_in_db = list_motions()
    print(motions_in_db)
    do_not_test = False
    already_tested = False
    if motions_in_db.motion_list:
        print("Non empty motiondb")
        if not ("dummymotion" in motions_in_db.motion_list):
            print("DB seems to contain real motions, exiting ...")
            do_not_test = True
        else:
            print("DB contains the dummymotion, we assume this database is a dummy database")
            already_tested = True

    else:
        print("Empty DB, proceeding with tests ...")

    # Exiting if we suspect this DB ain't a dummy one
    #print("\033[31m>>>>> Anti-data loss DEACTIVATED !!! <<<<<\033[39m")
    #do_not_test = False
    if do_not_test:
        exit()

    # -----------------------------------------
    # More info than true/false test outcome    
    display_info = False
    # Otherwise, we start tests:
    running_result = True
    fake_motion_data = [-0.56, -0.47, -0.51, -0.58,
                       -0.57, -0.52,  0.24,  0.48,
                        0.75,  0.75,  0.76, 0.76]
    if already_tested:
        # remove the dummy motion first
        result = delete_motion(motion_name = 'dummymotion')
        res = result.success
        running_result = running_result and res
        color_code = "\033[32m" if res else "\033[31m"
        result_string = "Deleting dummymotion returned: "+color_code
        print(result_string + "%r \033[39m" % res)

    # -----------------------------------------
    # Test storing:
    dummy_motion = Motion()
    dummy_motion.name = 'dummymotion'
    dummy_motion.type = "Trajectory"
    dummy_data = Float64MultiArray()
    dummy_data.layout = MultiArrayLayout()
    dummy_data.layout.dim = [
        MultiArrayDimension(label="points_dim0", size=4, stride= 0),
        MultiArrayDimension(label="points_dim1", size=4, stride= 0),
        MultiArrayDimension(label="points_dim2", size=4, stride= 0)]
    dummy_data.layout.data_offset = 0
    dummy_data.data = fake_motion_data
    dummy_motion.data = dummy_data
    result = store_motion(motion=dummy_motion)
    res = result.success
    running_result = running_result and res
    color_code = "\033[32m" if res else "\033[31m"
    result_string = "Storing dummymotion returned: "+color_code
    print(result_string + "%r \033[39m" % res)
    # -----------------------------------------

    motion = get_motion(name="dummymotion")
    res = motion.motion.name == "dummymotion"
    running_result = running_result and res
    color_code = "\033[32m" if res else "\033[31m"
    result_string = "Get dummymotion returned: "+color_code
    print(result_string + "%r \033[39m" % res)
    if display_info:
        print("Motion: %s" % motion.motion)
    # -----------------------------------------

    dummy_motion2 = Motion()
    dummy_motion2.name = 'dummymotion'
    dummy_motion2.type = "Trajectory"
    dummy_data2 = Float64MultiArray()
    dummy_data2.layout = MultiArrayLayout()
    dummy_data2.layout.dim = [
        MultiArrayDimension(label="points_dim0", size=4, stride= 0),
        MultiArrayDimension(label="points_dim1", size=4, stride= 0),
        MultiArrayDimension(label="points_dim2", size=4, stride= 0)]
    dummy_data2.layout.data_offset = 0
    dummy_data2.data = list(fake_motion_data)
    dummy_data2.data[::2] = [0.0] * (len(fake_motion_data)/2)
    dummy_motion2.data = dummy_data2
    
    result = update_motion(motion_name="dummymotion", motion=dummy_motion2)
    res = result.success
    running_result = running_result and res
    color_code = "\033[32m" if res else "\033[31m"
    result_string = "Update dummymotion returned: "+color_code
    print(result_string + "%r \033[39m" % res)
    # -----------------------------------------
    motion = get_motion(name="dummymotion")
    res = motion.motion.name == "dummymotion"
    running_result = running_result and res
    color_code = "\033[32m" if res else "\033[31m"
    result_string = "Get dummymotion returned: "+color_code
    print(result_string + "%r \033[39m" % res)
    if display_info:
        print("Motion: %s" % motion.motion)
    print("----------------------------------")
    print("MOTIONDB: Tests finished with"+"out \033[32merrors\033[39m" if running_result else " \033[31merrors\033[39m")

    # -----------------------------------------
    # =========================================
    # ADES
    print("==================================")
    # =========================================
    # Service proxies
    print("Testing Ades2DB")
    list_ades = rospy.ServiceProxy("ades2db/list_ades", ListAdes)
    delete_ades = rospy.ServiceProxy("ades2db/delete_ades", DeleteAdes)
    store_ades = rospy.ServiceProxy("ades2db/store_ades", StoreAdes)
    update_ades = rospy.ServiceProxy("ades2db/update_ades", UpdateAdes)
    get_ades_motions = rospy.ServiceProxy("ades2db/get_motions", GetAdesMotions)
    get_ades_motion_names = rospy.ServiceProxy("ades2db/get_motion_names", GetAdesMotionNames)
    get_preconds = rospy.ServiceProxy("ades2db/get_preconds", GetAdesPreConds)
    get_effects = rospy.ServiceProxy("ades2db/get_effects", GetAdesEffects)

    # Testing whether the DB is empty, if not raising a warning
    ades_in_db = list_ades()
    print(ades_in_db)
    do_not_test = False
    already_tested = False
    if ades_in_db.ades_list:
        print("Non empty adesdb")
        if not ("dummyades2" in ades_in_db.ades_list):
            print("DB seems to contain real ADES2, exiting ...")
            do_not_test = True
        else:
            print("DB contains the dummyades2, we assume this database is a dummy database")
            already_tested = True

    else:
        print("Empty DB, proceeding with tests ...")

    # Exiting if we suspect this DB ain't a dummy one
    print("\033[31m>>>>> Anti-data loss DEACTIVATED !!! <<<<<\033[39m")
    do_not_test = False
    if do_not_test:
        exit()

    # -----------------------------------------
    # Otherwise, we start tests:
    running_result = True
    
    if already_tested:
        # remove the dummy motion first
        result = delete_ades(ades_name = 'dummyades2')
        res = result.success
        running_result = running_result and res
        color_code = "\033[32m" if res else "\033[31m"
        result_string = "Deleting dummyades2 returned: "+color_code
        print(result_string + "%r \033[39m" % res)

    # -----------------------------------------
    # Test storing:
    dummy_ades = AdesData()
    dummy_ades.ades_name = 'dummyades2'
    dummy_ades.preconditions = [KeyValPair(key="isObject",value="(detected_target X)"), KeyValPair(key="isBreakable",value="(object O)")]
    dummy_ades.effects = [KeyValPair(key="dummied",value="(not (= object O)")]

    dummy_ms = MotionSequence()
    dummy_ms.sequence_name = "dummyms"
    dummy_ms.motions.append(dummy_motion)
    dummy_motion2.name = "another_dm"
    dummy_ms.motions.append(dummy_motion2)

    dummy_ades.motion_sequences.append(dummy_ms)

    result = store_ades(ades=dummy_ades)
    res = result.success
    running_result = running_result and res
    color_code = "\033[32m" if res else "\033[31m"
    result_string = "Storing dummyades2 returned: "+color_code
    print(result_string + "%r \033[39m" % res)
    # -----------------------------------------

    dummy_ms_motions = get_ades_motion_names(ades_name="dummyades2", motion_sequence="dummyms")
    #dummy_ades2 = get_ades_motions(name="dummyades2")
    res = (dummy_ms_motions.motion_names[0] == "dummymotion") and (dummy_ms_motions.motion_names[1] == "another_dm")
    running_result = running_result and res
    color_code = "\033[32m" if res else "\033[31m"
    result_string = "Get dummy motion names returned: "+color_code
    print(result_string + "%r \033[39m" % res)
    if display_info:
        print("Motion: %s" % dummy_ms_motions.motion_names)
    # -----------------------------------------
    updated_ades = AdesData(ades_name="dummyades2",preconditions=dummy_ades.preconditions, effects=dummy_ades.effects, motion_sequences=dummy_ades.motion_sequences)
    updated_ades.preconditions = []
    updated_ades.effects = []
    # motion_sequences[0] is dummyms, motions[0] is dummymotion
    updated_ades.motion_sequences[0].motions[0].data.data = [0.0] * len(fake_motion_data)
    print(updated_ades.motion_sequences[0].motions[0])
    result = update_ades(ades_name="dummyades2", ades=updated_ades)
    res = result.success
    running_result = running_result and res
    color_code = "\033[32m" if res else "\033[31m"
    result_string = "Update dummyades2 motion values returned: "+color_code
    print(result_string + "%r \033[39m" % res)

    # -----------------------------------------
    if False:
        motion = get_motion(name="dummymotion")
        res = motion.motion.name == "dummymotion"
        running_result = running_result and res
        color_code = "\033[32m" if res else "\033[31m"
        result_string = "Get dummymotion returned: "+color_code
        print(result_string + "%r \033[39m" % res)
        print("Motion: %s" % motion.motion)
        # -----------------------------------------
    
        print("Tests finished with"+"out \033[32merrors" if running_result else " \033[31merrors"+"\033[39m, exiting ...")
    # -----------------------------------------

    print("ADES2DB: Tests finished with"+"out \033[32merrors\033[39m" if running_result else " \033[31merrors\033[39m")
    # -----------------------------------------