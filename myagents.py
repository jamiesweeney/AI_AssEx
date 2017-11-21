"""
 Artificial Inteligence (H)
 Assessed Exercise 2017/2018

 Tested with Python 2.7

 Team 30
 Cameron Harper
 Jamie Sweeney
"""

#----------------------------------------------------------------------------------------------------------------#
class SolutionReport(object):
    # Please do not modify this custom class !
    def __init__(self):
        """ Constructor for the solution info
        class which is used to store a summary of
        the mission and agent performance """

        self.start_datetime_wallclock = None;
        self.end_datetime_wallclock = None;
        self.action_count = 0;
        self.reward_cumulative = 0;
        self.mission_xml = None;
        self.mission_type = None;
        self.mission_seed = None;
        self.student_guid = None;
        self.mission_xml_as_expected = None;
        self.is_goal = None;
        self.is_timeout = None;

    def start(self):
        self.start_datetime_wallclock = datetime.datetime.now()

    def stop(self):
        self.checkGoal()
        self.end_datetime_wallclock = datetime.datetime.now()

    def setMissionXML(self, mission_xml):
        self.mission_xml = mission_xml

    def setMissionType(self, mission_type):
        self.mission_type = mission_type

    def setMissionSeed(self, mission_seed):
        self.mission_seed = mission_seed

    def setStudentGuid(self, student_guid):
        self.student_guid = student_guid

    def addAction(self):
        self.action_count += 1

    def addReward(self, reward, datetime_):
        self.reward_cumulative += reward

    def checkMissionXML(self):
          """ This function is part of the final check"""
          #-- It is not included for simplifity --#
          self.mission_xml_as_expected = 'Unknown'

    def checkGoal(self):
          """ This function checks if the goal has been reached based on the expected reward structure (will not work if you change the mission xml!)"""
          #-- It is not included for simplifity --#
          if self.reward_cumulative!=None:
            x = round((abs(self.reward_cumulative)-abs(round(self.reward_cumulative)))*100);
            rem_goal = x%7
            rem_timeout = x%20
            if rem_goal==0 and x!=0:
                self.is_goal = True
            else:
                self.is_goal = False

            if rem_timeout==0 and x!=0:
                self.is_timeout = True
            else:
                self.is_timeout = False

#----------------------------------------------------------------------------------------------------------------#
class StateSpace(object):
    """ This is a datatype used to collect a number of important aspects of the environment
    It can be constructed online or be created offline using the Helper Agent

    You are welcome to modify or change it as you see fit

    """

    def __init__(self):
        """ Constructor for the local state-space representation derived from the Orcale"""
        self.state_locations = None;
        self.state_actions = None;
        self.start_id = None;  # The id assigned to the start state
        self.goal_id = None;  # The id assigned to the goal state
        self.start_loc = None; # The real word coordinates of the start state
        self.goal_loc = None; # The real word coordinates of the goal state
        self.reward_states_n = None
        self.reward_states = None
        self.reward_sendcommand = None
        self.reward_timeout = None
        self.timeout = None

#----------------------------------------------------------------------------------------------------------------#
def GetMissionInstance( mission_type, mission_seed, agent_type):
    """ Creates a specific instance of a given mission type """

    #Size of the problem
    msize = {
        'small': 10,
        'medium': 20,
        'large': 40,
    }

    # Timelimit
    mtimeout = {
        'small':   60000,
        'medium': 240000,
        'large':  960000,
        'helper': 100000,
    }

    # Number of intermediate rewards
    nir = {
        'small': 3,
        'medium': 9,
        'large': 27,
    }

    mission_type_tmp = mission_type
    if agent_type.lower()=='helper':
        mission_type_tmp = agent_type.lower()

    #-- Define various parameters used in the generation of the mission --#
    #-- HINT: It is crucial that you understand the details of the mission, this will require some knowledge of uncertainty/probability and random variables --#
    random.seed(mission_seed)
    reward_goal = abs(round(random.gauss(1000, 400)))+0.0700
    reward_waypoint = round(abs(random.gauss(3, 15)))
    reward_timeout = -round(abs(random.gauss(1000, 400)))-0.2000
    reward_sendcommand = round(-random.randrange(2,10))

    n_intermediate_rewards = random.randrange(1,5) * nir.get(mission_type, 10) # How many intermediate rewards...?

    xml_str = '''<?xml version="1.0" encoding="UTF-8" ?>
        <Mission xmlns="http://ProjectMalmo.microsoft.com" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
          <About>
            <Summary>Mission for assessed exercise 2016/2017 University of Glasgow</Summary>
          </About>
          <ServerSection>
            <ServerInitialConditions>
              <Time>
                <StartTime>6000</StartTime>
                <AllowPassageOfTime>false</AllowPassageOfTime>
              </Time>
              <Weather>clear</Weather>
              <AllowSpawning>false</AllowSpawning>
            </ServerInitialConditions>
            <ServerHandlers>
              <FlatWorldGenerator generatorString="3;7,220*1,5*3,2;3;,biome_1" />
              <MazeDecorator>
                <Seed>'''+str(mission_seed)+'''</Seed>
                <SizeAndPosition length="'''+str(msize.get(mission_type,100))+'''" width="'''+str(msize.get(mission_type,100))+'''" yOrigin="215" zOrigin="0" xOrigin="0" height="180"/>
                <GapProbability variance="0.3">0.2</GapProbability>
                <MaterialSeed>1</MaterialSeed>
                <AllowDiagonalMovement>false</AllowDiagonalMovement>
                <StartBlock fixedToEdge="true" type="emerald_block" height="1"/>
                <EndBlock fixedToEdge="false" type="redstone_block" height="12"/>
                <PathBlock type="glowstone" colour="WHITE ORANGE MAGENTA LIGHT_BLUE YELLOW LIME PINK GRAY SILVER CYAN PURPLE BLUE BROWN GREEN RED BLACK" height="1"/>
                <FloorBlock type="air"/>
                <SubgoalBlock type="glowstone"/>
                <GapBlock type="stained_hardened_clay" colour="WHITE ORANGE MAGENTA LIGHT_BLUE YELLOW LIME PINK GRAY SILVER CYAN PURPLE BLUE BROWN GREEN RED BLACK" height="3"/>
                <Waypoints quantity="'''+str(n_intermediate_rewards)+'''">
                  <WaypointItem type="diamond_block"/>
                </Waypoints>
              </MazeDecorator>
              <ServerQuitFromTimeUp timeLimitMs="'''+str(mtimeout.get(mission_type_tmp,0))+'''" description="out_of_time"/>
              <ServerQuitWhenAnyAgentFinishes />
            </ServerHandlers>
          </ServerSection>
          <AgentSection>
            <Name>My Agent</Name>
            <AgentStart>
              <Placement x="0" y="216" z="90"/> <!-- will be overwritten by MazeDecorator -->
            </AgentStart>
            <AgentHandlers>
              <ObservationFromFullStats/>
              <ObservationFromRecentCommands/>
              <ObservationFromFullInventory/>
              <RewardForCollectingItem>
                <Item reward="'''+str(reward_waypoint)+'''" type="diamond_block"/>
              </RewardForCollectingItem>
              <RewardForSendingCommand reward="'''+str(reward_sendcommand)+'''"/>
              <RewardForMissionEnd rewardForDeath="-1000000">
                <Reward description="found_goal" reward="'''+str(reward_goal)+'''" />
                <Reward description="out_of_time" reward="'''+str(reward_timeout)+'''" />
              </RewardForMissionEnd>
              <AgentQuitFromTouchingBlockType>
                <Block type="redstone_block" description="found_goal" />
              </AgentQuitFromTouchingBlockType>
            </AgentHandlers>
          </AgentSection>
        </Mission>'''
    return xml_str, msize.get(mission_type,100),reward_goal,reward_waypoint,n_intermediate_rewards,reward_timeout,reward_sendcommand,mtimeout.get(mission_type_tmp,0)

#----------------------------------------------------------------------------------------------------------------#
# This function initialized the mission based on the input arguments
def init_mission(agent_host, port=0, agent_type='Unknown',mission_type='Unknown', mission_seed=0, movement_type='Continuous'):
    """ Generate, and load the mission and return the agent host """

    #-- Set up the mission via XML definition --#
    mission_xml, msize, reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand, timeout = GetMissionInstance(mission_type,mission_seed,agent_type)
    my_mission = MalmoPython.MissionSpec(mission_xml, True)
    my_mission.forceWorldReset()

    #-- Enforce the specific restriction for the assessed exercise --#
    #-- If you want a super agent, define one for you self  --#
    my_mission.setModeToCreative()
    if agent_type.lower()=='random':
        n = msize
        my_mission.observeGrid(-n, -1, -n, n, -1, n, 'grid')
        my_mission.requestVideoWithDepth(320,240)
    elif agent_type.lower()=='simple':
        n = msize
        my_mission.observeGrid(-n, -1, -n, n, -1, n, 'grid');
        my_mission.requestVideo(320,240)
    elif agent_type.lower()=='realistic':
        n = 1 # n=1 means local info only !
        my_mission.observeGrid(-n,-1,-n, n, -1, n, 'grid');
        my_mission.requestVideoWithDepth(320,240)
    elif agent_type.lower()=='helper':
        n = 100
        my_mission.observeGrid(-n,-1,-n, n, -1, n, 'grid');
        my_mission.requestVideoWithDepth(320,240)
    else:
        #-- Define a custom agent and add the sensors you need --#
        n = 100
        my_mission.observeGrid(-n, -1, -n, n, 1, n, 'grid');
        my_mission.requestVideoWithDepth(320,240)

    #-- Add support for the specific movement type requested (and given the constraints of the assignment) --#
    #-- See e.g. http://microsoft.github.io/malmo/0.17.0/Schemas/MissionHandlers.html   --#
    if movement_type.lower()=='absolute':
        my_mission.allowAllAbsoluteMovementCommands()
    elif movement_type.lower()=='continuous':
        my_mission.allowContinuousMovementCommand('move')
        my_mission.allowContinuousMovementCommand('strafe')
        my_mission.allowContinuousMovementCommand('pitch')
        my_mission.allowContinuousMovementCommand('turn')
        my_mission.allowContinuousMovementCommand('crouch')
    elif movement_type.lower()=='discrete':
        my_mission.allowDiscreteMovementCommand('turn')
        my_mission.allowDiscreteMovementCommand('move')
        my_mission.allowDiscreteMovementCommand('movenorth')
        my_mission.allowDiscreteMovementCommand('moveeast')
        my_mission.allowDiscreteMovementCommand('movesouth')
        my_mission.allowDiscreteMovementCommand('movewest')
        my_mission.allowDiscreteMovementCommand('look')

    #-- Get the resulting xml (and return in order to check that conditions match the report) --#
    final_xml = my_mission.getAsXML(True)

    # Set up a recording for later inspection
    my_mission_record = MalmoPython.MissionRecordSpec('tmp' + ".tgz")
    my_mission_record.recordRewards()
    my_mission_record.recordMP4(24,400000)

    #-- Attempt to start a mission --#
    max_retries = 5
    for retry in range(max_retries):
        try:
            agent_host.startMission(my_mission, my_mission_record )
            break
        except RuntimeError as e:
            if retry == max_retries - 1:
                print("Error starting mission:",e)
                exit(1)
            else:
                time.sleep(2)

    #-- Loop until mission starts: --#
    print("Waiting for the mission to start ")
    state_t = agent_host.getWorldState()
    while not state_t.has_mission_begun:
        sys.stdout.write(".")
        time.sleep(0.1)
        state_t = agent_host.getWorldState()
        for error in state_t.errors:
            print("Error:",error.text)

    print
    print( "Mission started (xml returned)... ")
    return final_xml,reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand,timeout

#--------------------------------------------------------------------------------------
#-- This class implements the Realistic Agent --#
class AgentRealistic:

    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the realistic agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete' # HINT: You can change this if you want {Absolute, Discrete, Continuous}
        self.AGENT_NAME = 'Realistic'
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "movesouth 1", "movewest 1", "moveeast 1"]

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = None; # NOTE: The Realistic can not know anything about the state_space a prior i !
        self.solution_report = solution_report;   # Python is call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

        self.state_table = state_space_graph

    #-- Executes an action with a noisy transition model --#
    def __ExecuteActionForRealisticAgentWithNoisyTransitionModel__(self, idx_requested_action, noise_level):
        """ Creates a well-defined transition model with a certain noise level """
        n = len(self.AGENT_ALLOWED_ACTIONS)
        pp = noise_level/(n-1) * np.ones((n,1))
        pp[idx_requested_action] = 1.0 - noise_level
        idx_actual = np.random.choice(n, 1, p=pp.flatten()) # sample from the distribution of actions
        actual_action = self.AGENT_ALLOWED_ACTIONS[int(idx_actual)]
        self.agent_host.sendCommand(actual_action)
        return actual_action

    #-- Take a random action out of equally maximum actions from list --#
    def getAction(self, actions):
        max_val = max(actions)
        max_acts = []
        n = 0
        while n < len(actions):
            if (actions[n] == max_val):
                max_acts.append(n)
            n += 1

        action = random.randint(0,len(max_acts)-1)
        return max_acts[action]

    #-- Main agent logic --#
    def run_agent(self):

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        #-- Define local capabilities of the agent (sensors)--#
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)

        #-- Inital variables --#
        reward_cumulative = 0.0
        learn_rate = 0.5
        gamma = 0.95
        current_xpos = prev_xpos =  None
        current_zpos = prev_zpos = None
        current_key = prev_key = None
        first = True        #We ignore movement penalties from first movement, as it doesn't always work

        #-- Get inital state --#
        state_t = self.agent_host.getWorldState()

        #-- Get initial percepts --#
        while state_t.number_of_observations_since_last_state < 1:
            state_t = self.agent_host.getWorldState()
            time.sleep(0.1)
        msg = state_t.observations[-1].text
        oracle = json.loads(msg)
        current_xpos = str(int(oracle.get(u'XPos', 0)))
        current_zpos = str(int(oracle.get(u'ZPos', 0)))
        current_key = current_xpos + "_" + current_zpos

        #-- Add inital state to state table if not present --#
        if (current_key not in self.state_table):
            self.state_table[current_key] = [0,0,0,0]

        #-- Main loop --#
        while state_t.is_mission_running:

            #Take action
            actionIdx = None
            if (state_t.is_mission_running):
                actionIdx = self.getAction(self.state_table[current_key])
                print("Requested Action:",self.AGENT_ALLOWED_ACTIONS[actionIdx])

                # Now try to execute the action givne a noisy transition model
                actual_action = self.__ExecuteActionForRealisticAgentWithNoisyTransitionModel__(actionIdx, 0.1);
                print("Actual Action:",actual_action)

                #Add action to solution report
                actionIdx = self.AGENT_ALLOWED_ACTIONS.index(actual_action)
                self.solution_report.addAction()

            #Get post-action state
            time.sleep(0.2)
            state_t = self.agent_host.getWorldState()

            # Collect the number of rewards and add to reward_cumulative
            rewards = 0
            for reward_t in state_t.rewards:
                # Dont include timeout rewards
                #if (reward_t.getValue() > -700):
                rewards += reward_t.getValue()

                reward_cumulative += reward_t.getValue()
                self.solution_report.addReward(reward_t.getValue(), datetime.datetime.now())
                print("Reward_t:",reward_t.getValue())
                print("Cummulative reward so far:",reward_cumulative)
            # Check if anything went wrong along the way
            for error in state_t.errors:
                print("Error:",error.text)

            #Switch variables
            prev_xpos = current_xpos
            prev_zpos = current_zpos
            prev_key = current_key

            current_xpos = None
            current_zpos = None
            current_key = None

            # Get new percepts
            current_ypos = pitch = yaw = None
            while (state_t.is_mission_running and state_t.number_of_observations_since_last_state < 1):
                state_t = self.agent_host.getWorldState()

            if state_t.number_of_observations_since_last_state > 0:
                msg = state_t.observations[-1].text
                oracle = json.loads(msg)
                current_xpos = str(int(oracle.get(u'XPos', 0)))
                current_ypos = str(int(oracle.get(u'YPos', 0)))
                current_zpos = str(int(oracle.get(u'ZPos', 0)))
                pitch = str(int(oracle.get(u'Pitch', 0)))
                yaw = str(int(oracle.get(u'Yaw', 0)))
                current_key = current_xpos + "_" + current_zpos

            # If found a new state
            if (current_key != None and current_key not in self.state_table):
                self.state_table[current_key] = [0,0,0,0]

            # Add action result to state table
            if (current_key != None and actionIdx != None):
                self.state_table[prev_key][actionIdx] = (self.state_table[prev_key][actionIdx]) + (learn_rate)*(rewards + (gamma*max(self.state_table[current_key])) - self.state_table[prev_key][actionIdx])

            # Vision
            if state_t.number_of_video_frames_since_last_state > 0: # Have any Vision percepts been registred ?
                frame = state_t.video_frames[0]

            #-- Print some of the state information --#
            print("Percept: video,observations,rewards received:",state_t.number_of_video_frames_since_last_state,state_t.number_of_observations_since_last_state,state_t.number_of_rewards_since_last_state)
            print("\tcoordinates (x,y,z,yaw,pitch):" + str(current_xpos) + " " + str(current_ypos) + " " + str(current_zpos)+ " " + str(yaw) + " " + str(pitch))
            first = False


        # Summary
        print("Summary:")
        print("Cumulative reward = " + str(reward_cumulative) )
        return

#--------------------------------------------------------------------------------------
#-- This class implements the Simple Agent --#
class AgentSimple:

    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space):
        """ Constructor for the simple agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete' # HINT: You can change this if you want {Absolute, Discrete, Continuous}
        self.AGENT_NAME = 'Simple'
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "movesouth 1", "movewest 1", "moveeast 1"]

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = state_space;
        self.solution_report = solution_report;  # Python calls by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

    #-- Generates a list of actions from a shortest path --#
    def gen_actions(self, path):
        actions = []
        x,z = path[0].split('_')
        x, z = int(x), int(z)
        del path[0]
        for move in path:
            x2,z2 = move.split('_')
            x2,z2 = int(x2), int(z2)

            #Move north - in direction of negative z
            if (z2 < z):
                actions.append(0)
            #Move south - in direction of positive z
            elif (z2 > z):
                actions.append(1)
            #Move east - in direction of positive x
            elif (x2 > x):
                actions.append(3)
            #Move west - in direction of negative x
            elif (x2 < x):
                actions.append(2)

            x = x2
            z = z2
        return actions

    #-- Main agent logic --#
    def run_agent(self):

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

        #-- Inital variables --#
        reward_cumulative = 0.0
        graph = self.state_space['graph']       # state space graph
        start = self.state_space['start']       # start node
        end = self.state_space['end']           # end node
        path = None
        action_q = None

        #-- Get inital state --#
        state_t = self.agent_host.getWorldState()

        #-- Main loop --#
        while state_t.is_mission_running:
            # Get the world state
            state_t = self.agent_host.getWorldState()

            #If no path generated then make one
            if (path == None):
                def heuristic(a, b):
                    if 'in_path' in graph.node[a]:
                        return 0
                    else:
                        graph.node[a]['in_path'] = True
                        return (graph.node[a]['rewards']*(-1))
                path = nx.astar.astar_path(graph, start, end, heuristic=heuristic)
                action_q = self.gen_actions(path)

            #Take action
            if state_t.is_mission_running:

                #If no action in the queue - just wait
                if (action_q == None or len(action_q) == 0):
                    pass
                #If actions to take - take first
                else:
                    actionIdx = action_q[0]
                    del action_q[0]
                    print("Requested Action:",self.AGENT_ALLOWED_ACTIONS[actionIdx])

                    actual_action = self.AGENT_ALLOWED_ACTIONS[actionIdx]
                    self.agent_host.sendCommand(actual_action)
                    print("Actual Action:",actual_action)

                    #Add action to solution report
                    self.solution_report.addAction()
            time.sleep(0.2)

            state_t = self.agent_host.getWorldState()
            # Collect the number of rewards and add to reward_cumulative
            # Note: Since we only observe the sensors and environment every a number of rewards may have accumulated in the buffer
            for reward_t in state_t.rewards:
                reward_cumulative += reward_t.getValue()
                self.solution_report.addReward(reward_t.getValue(), datetime.datetime.now())
                print("Reward_t:",reward_t.getValue())
                print("Cummulative reward so far:",reward_cumulative)

            # Check if anything went wrong along the way
            for error in state_t.errors:
                print("Error:",error.text)

            # Handle the sensor input
            xpos = ypos = zpos = yaw = pitch = None
            if state_t.number_of_observations_since_last_state > 0: # Has any Oracle-like and/or internal sensor observations come in?
                msg = state_t.observations[-1].text      # Get the detailed for the last observed state
                oracle = json.loads(msg)                 # Parse the Oracle JSON

                # Orcale - get grid
                temp_grid = oracle.get(u'grid', 0)

                # GPS-like sensor
                xpos = oracle.get(u'XPos', 0)            # Position in 2D plane, 1st axis
                zpos = oracle.get(u'ZPos', 0)            # Position in 2D plane, 2nd axis (yes Z!)
                ypos = oracle.get(u'YPos', 0)            # Height as measured from surface! (yes Y!)

                # Standard "internal" sensory inputs
                yaw  = oracle.get(u'Yaw', 0)             # Yaw
                pitch = oracle.get(u'Pitch', 0)          # Pitch

            # Vision
            if state_t.number_of_video_frames_since_last_state > 0: # Have any Vision percepts been registred ?
                frame = state_t.video_frames[0]

            #-- Print some of the state information --#
            print("Percept: video,observations,rewards received:",state_t.number_of_video_frames_since_last_state,state_t.number_of_observations_since_last_state,state_t.number_of_rewards_since_last_state)
            print("\tcoordinates (x,y,z,yaw,pitch):" + str(xpos) + " " + str(ypos) + " " + str(zpos)+ " " + str(yaw) + " " + str(pitch))

        # Summary
        print("Summary:")
        print("Cumulative reward = " + str(reward_cumulative) )

        return

#--------------------------------------------------------------------------------------
#-- This class implements a basic, suboptimal Random Agent. The purpurpose is to provide a baseline for other agent to beat. --#
class AgentRandom:

    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the Random agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete'
        self.AGENT_NAME = 'Random'
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "movesouth 1", "movewest 1", "moveeast 1"]

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = state_space;
        self.solution_report = solution_report;   # Python makes call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

    def __ExecuteActionForRandomAgentWithNoisyTransitionModel__(self, idx_request_action, noise_level):
        """ Creates a well-defined transition model with a certain noise level """
        n = len(self.AGENT_ALLOWED_ACTIONS)
        pp = noise_level/(n-1) * np.ones((n,1))
        pp[idx_request_action] = 1.0 - noise_level
        idx_actual = np.random.choice(n, 1, p=pp.flatten()) # sample from the distrbution of actions
        actual_action = self.AGENT_ALLOWED_ACTIONS[int(idx_actual)] #int(idx_actual)
        self.agent_host.sendCommand(actual_action)
        return actual_action

    def run_agent(self):
        """ Run the Random agent and log the performance and resource use """

        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml,reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand, timeout = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)

        #-- Define local capabilities of the agent (sensors)--#
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)

        # Fix the randomness of the agent by seeding the random number generator
        reward_cumulative = 0.0

        # Main loop:
        state_t = self.agent_host.getWorldState()

        while state_t.is_mission_running:

            # Get the world state
            state_t = self.agent_host.getWorldState()

            #Take action
            if state_t.is_mission_running:
                actionIdx = random.randint(0, 3)
                print("Requested Action:",self.AGENT_ALLOWED_ACTIONS[actionIdx])

                # Now try to execute the action givne a noisy transition model
                # Set at 0 noise just now as actions are noise free
                actual_action = self.__ExecuteActionForRandomAgentWithNoisyTransitionModel__(actionIdx, 0.00);
                print("Actual Action:",actual_action)

                #Add action to solution report
                self.solution_report.addAction()

            time.sleep(0.2)
            state_t = self.agent_host.getWorldState()

            # Collect the number of rewards and add to reward_cumulative
            # Note: Since we only observe the sensors and environment every a number of rewards may have accumulated in the buffer
            for reward_t in state_t.rewards:
                reward_cumulative += reward_t.getValue()
                self.solution_report.addReward(reward_t.getValue(), datetime.datetime.now())
                print("Reward_t:",reward_t.getValue())
                print("Cummulative reward so far:",reward_cumulative)

            # Check if anything went wrong along the way
            for error in state_t.errors:
                print("Error:",error.text)

            # Handle the sensor input
            xpos = ypos = zpos = yaw = pitch = None
            if state_t.number_of_observations_since_last_state > 0: # Has any Oracle-like and/or internal sensor observations come in?
                msg = state_t.observations[-1].text      # Get the detailed for the last observed state
                oracle = json.loads(msg)                 # Parse the Oracle JSON

                # Orcale
                grid = oracle.get(u'grid', 0)            #

                # GPS-like sensor
                xpos = oracle.get(u'XPos', 0)            # Position in 2D plane, 1st axis
                zpos = oracle.get(u'ZPos', 0)            # Position in 2D plane, 2nd axis (yes Z!)
                ypos = oracle.get(u'YPos', 0)            # Height as measured from surface! (yes Y!)

                # Standard "internal" sensory inputs
                yaw  = oracle.get(u'Yaw', 0)             # Yaw
                pitch = oracle.get(u'Pitch', 0)          # Pitch

            # Vision
            if state_t.number_of_video_frames_since_last_state > 0: # Have any Vision percepts been registred ?
                frame = state_t.video_frames[0]

            #-- Print some of the state information --#
            print("Percept: video,observations,rewards received:",state_t.number_of_video_frames_since_last_state,state_t.number_of_observations_since_last_state,state_t.number_of_rewards_since_last_state)
            print("\tcoordinates (x,y,z,yaw,pitch):" + str(xpos) + " " + str(ypos) + " " + str(zpos)+ " " + str(yaw) + " " + str(pitch))

        # --------------------------------------------------------------------------------------------
        # Summary
        print("Summary:")
        print("Cumulative reward = " + str(reward_cumulative) )

        return

#--------------------------------------------------------------------------------------
#-- This class implements a helper Agent for deriving the state-space representation ---#
class AgentHelper:
    """ This agent determines the state space for use by the actual problem solving agent. Enabeling do_plot will allow you to visualize the results """

    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the helper agent """
        self.AGENT_NAME = 'Helper'
        self.AGENT_MOVEMENT_TYPE = 'Absolute' # Note the helper needs absolute movements
        self.DO_PLOT = False

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type
        self.state_space = {}
        self.solution_report = solution_report;   # Python is call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)


    def add_node(self, graph, block, i, j):
        code = str(i) + "_" + str(j)
        node = graph.add_node(code)
        #Add paths
        if (graph.has_node(str(i-1) + "_" + str(j))):
            graph.add_edge(code, str(i-1) + "_" + str(j), weight=9)
        if (graph.has_node(str(i) + "_" + str(j-1))):
            graph.add_edge(code, str(i) + "_" + str(j-1), weight=9)
        return code


    def run_agent(self):
        """ Run the Helper agent to get the state-space """

        #-- Load and init the Helper mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml,reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand, timeout = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)

        #-- Define local capabilities of the agent (sensors)--#
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)

        time.sleep(1)

        #-- Get the state of the world along with internal agent state...--#
        state_t = self.agent_host.getWorldState()

        #-- Get a state-space model by observing the Orcale/GridObserver--#
        if state_t.is_mission_running:
            #-- Make sure we look in the right direction when observing the surrounding (otherwise the coordinate system will rotated by the Yaw !) --#
            # Look East (towards +x (east) and +z (south) on the right, i.e. a std x,y coordinate system) yaw=-90
            self.agent_host.sendCommand("setPitch 20")
            time.sleep(1)
            self.agent_host.sendCommand("setYaw 0")
            time.sleep(1)

            #-- Basic map --#
            state_t = self.agent_host.getWorldState()

            #Wait for percepts
            while state_t.number_of_observations_since_last_state == 0:
                state_t = self.agent_host.getWorldState()
                time.sleep(1)

            #Get coords and map of state state_space
            msg = state_t.observations[-1].text
            oracle_and_internal = json.loads(msg)
            temp_grid = oracle_and_internal.get(u'grid', 0)
            xpos = int(oracle_and_internal.get(u'XPos', 0))
            ypos = int(oracle_and_internal.get(u'YPos', 0))
            zpos = int(oracle_and_internal.get(u'ZPos', 0))

            #Change grid to 2d array
            i = 0
            grid = []

            col_size = int(math.sqrt(len(temp_grid)))
            while (i < col_size):
                row = temp_grid[(i*col_size):((i+1)*col_size)]
                grid.append([])
                grid[i] = row
                i = i + 1

            #Create graph
            graph = nx.Graph()
            i = 0
            j = 0
            start = end = None
            while (i < col_size):
                row = grid[i]
                while (j < col_size):
                    cell = row[j]

                    x_val =(j-int((col_size/2))) + xpos
                    z_val =(i-int((col_size/2))) + zpos
                    #If to be added, add, then add paths
                    if (cell in ['glowstone', 'emerald_block', 'redstone_block']):
                        node = self.add_node(graph, cell, x_val, z_val)
                        if (cell == 'emerald_block'):
                            start = node
                        if (cell == 'redstone_block'):
                            end = node

                    j = j + 1
                j = 0
                i = i + 1

            #For each node, teleport to it and check if rewards recieved
            for node in graph.nodes:
                graph.node[node]['rewards'] = -9
                if not (str(node) == end):
                    x_val = node.split('_')[0]
                    z_val = node.split('_')[1]
                    #Teleport
                    agent_host.sendCommand("tp " + str(x_val) + ".5 " + str(ypos) + ".5 " + str(z_val) + ".5")
                    #Collect rewards
                    state_t = self.agent_host.getWorldState()
                    for reward_t in state_t.rewards:
                        graph.node[node]['rewards'] = graph.node[node]['rewards'] + reward_t.getValue()
                    time.sleep(1)

            #Finish mission
            agent_host.sendCommand("tp " + str(0 ) + " " + str(0) + " " + str(0))
            time.sleep(0.5)

            #-- Save the info  --
            self.state_space['graph'] = graph
            self.state_space['start'] = start
            self.state_space['end'] = end

        #Wait if mission has not finished
        while state_t.is_mission_running:
                state_t = self.agent_host.getWorldState()
                time.sleep(5)
        return


# --------------------------------------------------------------------------------------------
#-- The main entry point if you run the module as a script--#
if __name__ == "__main__":
    import os
    #-- Define default arguments, in case you run the module as a script --#
    DEFAULT_STUDENT_GUID = 'template'
    DEFAULT_AGENT_NAME   = 'Random' #Choose between {Random, Simple, Realistic}
    DEFAULT_MALMO_PATH   = os.environ["MALMO_ROOT"] # HINT: Change this to your own path
    DEFAULT_AIMA_PATH    = '/Users/ttrx/Documents/GitHub/AI_AssEx'  # HINT: Change this to your own path, forward slash only, should be the 2.7 version from https://www.dropbox.com/s/vulnv2pkbv8q92u/aima-python_python_v27_r001.zip?dl=0) or for Python 3.x get it from https://github.com/aimacode/aima-python
    DEFAULT_MISSION_TYPE = 'small'  #Choose between {small,medium,large}
    DEFAULT_MISSION_SEED_MAX = 1    #How many different instances of the given mission (i.e. maze layout)
    DEFAULT_REPEATS      = 1        #How many repetitions of the same maze layout
    DEFAULT_PORT         = 0
    DEFAULT_SAVE_PATH    = './results/'

    #-- Import required modules --#
    import sys
    import time
    import random
    from random import shuffle
    import json
    import argparse
    import pickle
    import datetime
    import math
    import numpy as np
    from copy import deepcopy
    import hashlib
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    import networkx as nx
    from matplotlib import lines

    #-- Define the commandline arguments required to run the agents from command line --#
    parser = argparse.ArgumentParser()
    parser.add_argument("-a" , "--agentname"        , type=str, help="path for the malmo pyhton examples"   , default=DEFAULT_AGENT_NAME)
    parser.add_argument("-t" , "--missiontype"      , type=str, help="mission type (small,medium,large)"    , default=DEFAULT_MISSION_TYPE)
    parser.add_argument("-s" , "--missionseedmax"   , type=int, help="maximum mission seed value (integer)"           , default=DEFAULT_MISSION_SEED_MAX)
    parser.add_argument("-n" , "--nrepeats"         , type=int, help="repeat of a specific agent (if stochastic behavior)"  , default=DEFAULT_REPEATS)
    parser.add_argument("-g" , "--studentguid"      , type=str, help="student guid"                         , default=DEFAULT_STUDENT_GUID)
    parser.add_argument("-p" , "--malmopath"        , type=str, help="path for the malmo pyhton examples"   , default=DEFAULT_MALMO_PATH)
    parser.add_argument("-x" , "--malmoport"        , type=int, help="special port for the Minecraft client", default=DEFAULT_PORT)
    parser.add_argument("-o" , "--aimapath"         , type=str, help="path for the aima toolbox (optional)"   , default=DEFAULT_AIMA_PATH)
    parser.add_argument("-r" , "--resultpath"       , type=str, help="the path where the results are saved" , default=DEFAULT_SAVE_PATH)
    args = parser.parse_args()
    print(args)

    #-- Display info about the system --#
    print("Working dir:"+os.getcwd())
    print("Python version:"+sys.version)
    print("malmopath:"+args.malmopath)
    print("JAVA_HOME:'"+os.environ["JAVA_HOME"]+"'")
    print("MALMO_XSD_PATH:'"+os.environ["MALMO_XSD_PATH"]+"'")
    print("MALMO_ROOT: " + os.environ["MALMO_ROOT"])
    #-- Add the Malmo path  --#
    print('Add Malmo Python API/lib to the Python environment ['+args.malmopath+'/Python_Examples'+']')
    sys.path.append(args.malmopath+'Python_Examples/')

    #-- Import the Malmo Python wrapper/module --#
    print('Import the Malmo module...')
    import MalmoPython

    #-- OPTIONAL: Import the AIMA tools (for representing the state-space)--#
    #print('Add AIMA lib to the Python environment ['+args.aimapath+']')
    #sys.path.append(args.aimapath+'/')
    #from search import *

    #-- Create the command line string for convenience --#
    cmd = 'python myagents.py -a ' + args.agentname + ' -s ' + str(args.missionseedmax) + ' -n ' + str(args.nrepeats) + ' -t ' + args.missiontype + ' -g ' + args.studentguid + ' -p ' + args.malmopath + ' -x ' + str(args.malmoport)
    print(cmd)

    #-- Run the agent a number of times (it only makes a difference if you agent has some random elemnt to it, initalizaiton, behavior, etc.) --#
    #-- HINT: It is quite important that you understand the need for the loops  --#
    #-- HINT: Depending on how you implement your realistic agent in terms of restarts and repeats, you may want to change the way the loops operate --#

    print('Instantiate an agent interface/api to Malmo')
    agent_host = MalmoPython.AgentHost()
    totals = []

    #-- Itereate a few different layout of the same mission stype --#
    for i_training_seed in range(0,args.missionseedmax):

        #-- Observe the full state space a prior i (only allowed for the simple agent!) ? --#
        if args.agentname.lower()=='simple':
            print('Get state-space representation using a AgentHelper...[note in v0.30 there is now an faster way of getting the state-space ]')
            helper_solution_report = SolutionReport()
            helper_agent = AgentHelper(agent_host,args.malmoport,args.missiontype,i_training_seed, helper_solution_report, None)
            helper_agent.run_agent()
            time.sleep(2)
        else:
            helper_agent = None

        #-- Repeat the same instance (size and seed) multiple times --#
        persistant_state_space = {}
        for i_rep in range(0,args.nrepeats):
            print('Setup the performance log...')
            solution_report = SolutionReport()
            solution_report.setStudentGuid(args.studentguid)

            print('Get an instance of the specific ' + args.agentname + ' agent with the agent_host and load the ' + args.missiontype + ' mission with seed ' + str(i_training_seed))
            agent_name = 'Agent' + args.agentname
            state_space = None;
            if not helper_agent==None:
                state_space = deepcopy(helper_agent.state_space)

            # If realistic, keep state table from last cycle
            if args.agentname.lower()=='realistic':
                state_space = persistant_state_space

            agent_to_be_evaluated = eval(agent_name+'(agent_host,args.malmoport,args.missiontype,i_training_seed,solution_report,state_space)')

            print('Run the agent, time it and log the performance...')
            solution_report.start() # start the timer (may be overwritten in the agent to provide a fair comparison)
            agent_to_be_evaluated.run_agent()
            solution_report.stop() # stop the timer

            # If realistic update persistant state space
            if args.agentname.lower()=='realistic':
                persistant_state_space = agent_to_be_evaluated.state_table

            #
            totals.append(int(solution_report.reward_cumulative))

            print("\n---------------------------------------------")
            print("| Solution Report Summary: ")
            print("|\tCumulative reward = " + str(solution_report.reward_cumulative))
            print("|\tDuration (wallclock) = " + str((solution_report.end_datetime_wallclock-solution_report.start_datetime_wallclock).total_seconds()))
            print("|\tNumber of reported actions = " + str(solution_report.action_count))
            print("|\tFinal goal reached = " + str(solution_report.is_goal))
            print("|\tTimeout = " + str(solution_report.is_timeout))
            print("---------------------------------------------\n")

            print('Save the solution report to a specific file for later analysis and reporting...')
            fn_result = args.resultpath + 'solution_' + args.studentguid + '_' + agent_name + '_' +args.missiontype + '_' + str(i_training_seed) + '_' + str(i_rep)
            foutput = open(fn_result+'.pkl', 'wb+')
            pickle.dump(agent_to_be_evaluated.solution_report,foutput) # Save the solution information in a specific file, HiNT:  It can be loaded with pickle.load(output) with read permissions to the file
            foutput.close()

            # You can reload the results for this instance using...
            #finput = open(fn_result+'.pkl', 'rb')
            #res =  pickle.load(finput)
            #finput.close()

            print('Sleep a sec to make sure the client is ready for next mission/agent variation...')
            time.sleep(1)
            print ("Iteration: " + str(i_rep))
            print("------------------------------------------------------------------------------\n")



    print("Done")
    print (totals)
    plt.plot(totals)
    plt.show()
