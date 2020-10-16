#!/usr/bin/env python

import rospy
import rosservice
import numpy as np
from imagine_common.srv import ListAdes, GetAdesPreConds, GetAdesEffects, GetAdesMotions, GetAdesMotionNames, StoreAdes, UpdateAdes, DeleteAdes, ListMotion, GetMotion, StoreMotion, UpdateMotion, DeleteMotion, UpdateAdesMotionNames, AddMotionSequence
from imagine_common.msg import Motion, KeyValPair, AdesData, MotionSequence
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension, String

if __name__ == "__main__":
    rospy.init_node("merge_symb_subsymb_db")

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
    if type_symb == '':
        type_symb = default_type_symb
    else:
        type_symb = int(type_symb)

    adesdb_name = "/ades2db" if type_symb == 2 else "/adesdb"
    print("Waiting for symbolic db to be up")
    srv_ades_symb = namespace_symb + adesdb_name
    srv_motion_symb = namespace_symb + "/motiondb"
    
    print("Expected service:"+srv_ades_symb+"/list_ades")
    rospy.wait_for_service(srv_ades_symb+"/list_ades")
    if type_symb == 2:
        rospy.wait_for_service(srv_motion_symb+"/list_motion")

    if not namespace_subs:
        namespace_subs = default_namespace_subs
    if type_subs == '':
        type_subs = default_type_subs
    else:
        type_subs = int(type_subs)

    adesdb_name = "/ades2db" if type_subs == 2 else "/adesdb"
    print("Waiting for subsymbolic db to be up")
    srv_ades_subs = namespace_subs + adesdb_name
    srv_motion_subs = namespace_subs + "/motiondb"
    
    print("Expected service:"+srv_ades_subs+"/list_ades")
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
    - For each subsymbolic ADES
        - get motion and try to store them
        - if already there, ask for update or ignore
    - for each symbolic ADES
        - store PC/eff in new ADEs
        - get motion sequences if any
            - check whether the motions of MS are in motion DB
            - if not, skip the motion/ask for which existing motion
        - when all motion sequences are correct, store new ADES
    '''

    # Symbolic services proxies
    list_ades_symb = rospy.ServiceProxy(srv_ades_symb+"/list_ades", ListAdes)
    get_preconds_symb = rospy.ServiceProxy(srv_ades_symb+"/get_preconds", GetAdesPreConds)
    get_effects_symb = rospy.ServiceProxy(srv_ades_symb+"/get_effects", GetAdesEffects)
    get_motions_symb = rospy.ServiceProxy(srv_ades_symb+"/get_motions", GetAdesMotions)
    
    # subsymbolic services proxies
    list_ades_subs = rospy.ServiceProxy(srv_ades_subs+"/list_ades", ListAdes)
    get_ades_motions_subs = rospy.ServiceProxy(srv_ades_subs+"/get_motions", GetAdesMotions)
    get_ades_motion_names_subs = rospy.ServiceProxy(srv_ades_subs+"/get_motion_names", GetAdesMotionNames)
    get_motion_subs = rospy.ServiceProxy(srv_motion_subs+"/get_motion", GetMotion)
    
    # Target services proxies
    store_ades_targ = rospy.ServiceProxy(srv_ades_targ+"/store_ades", StoreAdes)
    update_ades_motion_targ = rospy.ServiceProxy(srv_ades_targ+"/update_ades_motion", UpdateAdesMotionNames)
    add_motion_sequence_targ = rospy.ServiceProxy(srv_ades_targ+"/add_motion_sequence", AddMotionSequence)
    list_motion_targ = rospy.ServiceProxy(srv_motion_targ+"/list_motion", ListMotion)
    store_motion_targ = rospy.ServiceProxy(srv_motion_targ+"/store_motion", StoreMotion)

    # CONTINUE THAT
    # Check that dbs contain the same ADES
    ades_symb = list_ades_symb().ades_list
    ades_subs = list_ades_subs().ades_list
    len_symb = len(ades_symb)
    len_subs = len(ades_subs)

    current_motions_targ = list_motion_targ().motion_list
    print("Lengths SY: %d, SB: %d" % (len_symb, len_subs))
    for ad_sub in ades_subs:
        print(ad_sub)
        new_motions = get_ades_motions_subs(ad_sub).motion_sequences
        print(len(new_motions))
        for ms in new_motions:
            print(ms.sequence_name)
            print(ms.score)
            print(len(ms.motions))
            for m in ms.motions:
                if len(ms.motions) == 1 and len(new_motions) == 1:
                    print(m.name)
                    print(m.type)
                    m.name = ad_sub
                    print(m.name)
                    store_motion_targ(m)
                else:
                    print("motion ignored")
            # the following only works with type2 database
            #motions_subs = get_ades_motion_names_subs(ad_sub, ms.sequence_name).motion_names
            #print(motions_subs)
            #for m in motions_subs:
            #    if m in current_motions_targ:
            #        print(m+" is a known motion")
            #        print("not stored")
            #    else:
            #        print(m+" is unknwon")
            #        m_subs = get_motion_subs(m)
            #        print(type(m_subs.motion))
            #        print(m_subs)
            #        store_motion_targ(m_subs.motion)
        print("===============================")

    # all motions have been stored, now move on to the 
    # symbolic part
    for ad_sym in ades_symb:
        print(ad_sym)
        preconds_symb = get_preconds_symb(ad_sym)
        effects_symb = get_effects_symb(ad_sym)
        motions_symb = get_motions_symb(ad_sym)
        msms = motions_symb.motion_sequences
        check_motion_sequences = False
        if msms:
            ms_names=[ms.sequence_name for ms in msms]
            print(ms_names)
            if ms_names[0] == '' and len(ms_names) == 1:
                print("Only one empty motion sequence, ignoring it")
            else:
                print("Existing motion sequences for symb ADES")
                check_motion_sequences = True

        else:
            print("NO existing motion sequences for symb ADES")
            print(msms)

        # TODO here
        if check_motion_sequences:
            pass
            # make user check the existing MS and/or modify them in necessary
        else:
            # make user create MS for this ADES
            add_new_ms = True
            new_sequences = []
            motions_per_sequence = {}
            while add_new_ms:
                res = raw_input("Enter new motion sequence name (or q to exit): ")
                if res == 'q':
                    add_new_ms = False
                    break
                new_ms = MotionSequence()
                new_ms.sequence_name = res
                res = raw_input("Enter new motion sequence score: ")
                new_ms.score = float(res)
                res = raw_input("Enter new motion sequence input types: ")
                new_ms.input_types = res.split(' ')
                res = raw_input("Enter new motion sequence motions name (separated by ' '): ")
                print("Motion per sequence")
                motions_per_sequence[new_ms.sequence_name] = [m for m in res.split(' ') if m != '']
                print(motions_per_sequence)
                #new_ms.score = res.split(' ')
                new_sequences.append(new_ms)
            
            new_ades = AdesData()
            new_ades.ades_name = ad_sym
            #print(type())
            new_ades.preconditions = preconds_symb.preconditions
            new_ades.effects = effects_symb.effects
            new_ades.motion_sequences = new_sequences
            store_ades_targ(ades=new_ades)
            # set ades motion names
            for ms in new_sequences:
                add_motion_sequence_targ(ades_name=ad_sym, motion_sequence=ms)
            for k, v in motions_per_sequence.iteritems():
                update_ades_motion_targ(ades_name=new_ades.ades_name, sequence_name=k, sequence_motion_names=v)
