#!/usr/bin/env python

import rospy
import rosservice
import numpy as np
from imagine_common.srv import ListAdes, GetAdesPreConds, GetAdesEffects, GetAdesMotions, GetAdesMotionNames, StoreAdes, UpdateAdes, DeleteAdes, ListMotion, GetMotion, StoreMotion, UpdateMotion, DeleteMotion
from imagine_common.msg import Motion, KeyValPair, AdesData, MotionSequence
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension, String

if __name__ == "__main__":
    rospy.init_node("merge_multi_db")

    print("Merging Symbolic part of one AdesDB to the Subsymbolic part of another AdesDB.")
    print("This process requires to run in parallel 3 adesdb/ades2db+motiondb : 2 with the same ADESes but partial info, 1 as the target database")
    print("Start the databases within ros namespaces")

    choice = raw_input("Do you want to proceed? [Y/n]: ")
    if choice == 'n':
        exit()

    default_namespace_symb = "symb"
    default_namespace_subs = "subsymb"
    default_namespace_targ = "target"
    default_type_symb = 2
    default_type_subs = 2
    # target is always 2, we want to get rid of db1

    namespace_symb = raw_input("Enter namespace for symbolic db (default:"+default_namespace_symb+"): ")
    type_symb = raw_input("Enter type (default:"+str(default_type_symb)+"): ")
    
    namespace_subs = raw_input("Enter namespace for subsymbolic db (default:"+default_namespace_subs+"): ")
    type_subs = raw_input("Enter type (default:"+str(default_type_subs)+"): ")
    
    namespace_targ = raw_input("Enter namespace of target DB (default:"+default_namespace_targ+"): ")

    # ------- this whole code block can probably be improved
    #if db1, wait for ns/adesdb/list_ades
    #if db2, wait for ns/ades2db/list_ades and ns/motiondb/list_motion
    if not namespace_symb:
        namespace_symb = default_namespace_symb
    if not type_symb:
        type_symb = default_type_symb

    print("Waiting for symbolic db to be up")
    srv_ades_symb = namespace_symb + "/ades2db" if type_symb == 2 else "/adesdb"
    srv_motion_symb = namespace_symb + "/motiondb"
    
    rospy.wait_for_service(srv_ades_symb+"/list_ades")
    if type_symb == 2:
        rospy.wait_for_service(srv_motion_symb+"/list_motion")

    if not namespace_subs:
        namespace_subs = default_namespace_subs
    if not type_subs:
        type_subs = default_type_subs

    print("Waiting for subsymbolic db to be up")
    srv_ades_subs = namespace_subs + "/ades2db" if type_subs == 2 else "/adesdb"
    srv_motion_subs = namespace_subs + "/motiondb"
    
    rospy.wait_for_service(srv_ades_subs+"/list_ades")
    if type_subs == 2:
        rospy.wait_for_service(srv_motion_subs+"/list_motion")

    print("Waiting for target db to be up")
    if not namespace_targ:
        namespace_targ = default_namespace_targ
    srv_ades_targ = namespace_targ + "/ades2db"
    srv_motion_targ = namespace_targ + "/motiondb"
    rospy.wait_for_service(srv_ades_targ+"/list_ades")
    rospy.wait_for_service(srv_motion_targ+"/list_motion")
    # -----------------------------------

    '''
    # Pseudo code:
    - assume default symb/subsymb namespaces and check there are running dbs there
    - if yes, proceed, else, ask for namespaces
    - Check that both databases are running (by wait_for ades_listing services)
    - Check that they contain the same ADESes
    - ask for the target database???
    - For each ADES pair
        - create a new one set with values from both ADES of the pair
        - store new ADES
    '''

    # Symbolic services proxies
    list_ades_symb = rospy.ServiceProxy(srv_ades_symb+"/list_ades", ListAdes)
    #get_ades_motions = rospy.ServiceProxy(srv_ades_symb+"/get_motions", GetAdesMotions)
    get_preconds_symb = rospy.ServiceProxy(srv_ades_symb+"/get_preconds", GetAdesPreConds)
    get_effects_symb = rospy.ServiceProxy(srv_ades_symb+"/get_effects", GetAdesEffects)
    #list_motions_symb = rospy.ServiceProxy(srv_motion_symb+"/list_motion", ListMotion)
    #get_motion = rospy.ServiceProxy("motiondb/get_motion", GetMotion)
    #list_ades = rospy.ServiceProxy("ades2db/list_ades", ListAdes)
    #store_ades = rospy.ServiceProxy("ades2db/store_ades", StoreAdes)
    #get_ades_motion_names = rospy.ServiceProxy("ades2db/get_motion_names", GetAdesMotionNames)
    
    # subsymbolic services proxies
    list_ades_subs = rospy.ServiceProxy(srv_ades_subs+"/list_ades", ListAdes)
    get_ades_motions_subs = rospy.ServiceProxy(srv_ades_subs+"/get_motions", GetAdesMotions)
    #get_preconds = rospy.ServiceProxy(srv_ades_symb+"/get_preconds", GetAdesPreConds)
    #get_effects = rospy.ServiceProxy(srv_ades_symb+"/get_effects", GetAdesEffects)
    
    # Target services proxies
    store_ades_targ = rospy.ServiceProxy(srv_ades_targ+"/store_ades", StoreAdes)

    # Check that dbs contain the same ADES
    ades_symb = list_ades_symb().ades_list
    ades_subs = list_ades_subs().ades_list
    len_symb = len(ades_symb)
    len_subs = len(ades_subs)
    #print("Length test: %r" % (len_symb == len_subs))
    print("Lengths SY: %d, SB: %d" % (len_symb, len_subs))
    if len_symb != len_subs:
        print("databases contain a different number of ADES.")
        choice = raw_input("Do you want to proceed? [Y/n]: ")
        if choice == 'n':
            exit()

    common_ades = set()
    if ades_symb != ades_subs:
        print("Databases contain different ADES.")
        print(ades_symb)
        print(ades_subs)
        common_ades = set(ades_symb).intersection(ades_subs)
        print("There are only "+str(len(common_ades))+" in common.")
        choice = raw_input("Do you want to proceed? [Y/n]: ")
        if choice == 'n':
            exit()

    print("Common ades nb: %d" % len(common_ades))
    for a_s in common_ades:
        # fetch infos:
        new_pc = get_preconds_symb(a_s)
        new_ef = get_effects_symb(a_s)
        new_motions = get_ades_motions_subs(a_s)
        print(new_pc)
        print(new_ef)
        print(new_motions)

    exit()
    # Service proxies
    '''
    print("Testing MotionDB")
    #list_motions = rospy.ServiceProxy("motiondb/list_motion", ListMotion)
    #delete_motion = rospy.ServiceProxy("motiondb/delete_motion", DeleteMotion)
    #store_motion = rospy.ServiceProxy("motiondb/store_motion", StoreMotion)
    #update_motion = rospy.ServiceProxy("motiondb/update_motion", UpdateMotion)
    #delete_ades = rospy.ServiceProxy("ades2db/delete_ades", DeleteAdes)
    #update_ades = rospy.ServiceProxy("ades2db/update_ades", UpdateAdes)
    #get_motion = rospy.ServiceProxy("motiondb/get_motion", GetMotion)


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
    '''