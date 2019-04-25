import rospy
import rostopic
from std_msgs.msg import Header, String
from sets import ImmutableSet
from sets import Set
import threading
import hsm
import pyhsm_msgs.msg as msgs

__all__ = ['IntrospectionServer']

# Topic names
STATUS_TOPIC = '/pyhsm/status'
INIT_TOPIC = '/pyhsm/init'
STRUCTURE_TOPIC = '/pyhsm/structure'
TRANSITION_TOPIC = '/pyhsm/transition'
EVENT_TOPIC = '/pyhsm/event'

import hsm.core
from hsm.core import Container

class IntrospectionClient():
     def get_servers(self):
         """Get the base names that are broadcasting pyhsm states."""
         # Get the currently broadcasted pyhsm introspection topics
         topics = rostopic.find_by_type('pyhsm_msgs/HsmStatus')
         return [t[:t.rfind(STATUS_TOPIC)] for t in topics] # returns ['/pyhsm_introspection']
         
#     def set_initial_server_state(self,
#                           server,
#                           path,
#                           initial,
#                           timeout=None):
#         """Set the initial state of a pyhsm server.
#
#         @type server: string
#         @param server: The name of the introspection server to which this client
#         should connect.
#
#         @type path: string
#         @param path: The path to the target container in the state machine.
#
#         @type initial_states: list of string
#         @param inital_state: The state the target container should take when it
#         starts. This is as list of at least one state label.
#
#         @type initial_userdata: UserData
#         @param initial_userdata: The userdata to inject into the target container.
#
#         @type timeout: rospy.Duration
#         @param timeout: Timeout for this call. If this is set to None, it will not
#         block, and the initial state may not be set before the target state machine
#         goes active.
#         """
#
#         # Construct initial state command
#         initial_status_msg = msgs.ContainerInitialStatusCmd(
#             path=path,
#             initial=initial)
#         print('Init_status_msg: ' + str(initial_status_msg))
#
#         # A status message to receive confirmation that the state was set properly
#         msg_response = msgs.HsmStatus()
#
#         # Define a local callback to just stuff a local message
#         def local_cb(msg, msg_response):
#             rospy.logdebug("Received status response: " + str(msg))
#             msg_response.path = msg.path
#             msg_response.current = msg.initial
#
#         # Create a subscriber to verify the request went through
#         state_sub = rospy.Subscriber(server + STATUS_TOPIC, msgs.HsmStatus,
#                                      callback=local_cb, callback_args=msg_response)
#
#         # Create a publisher to send the command
#         rospy.logdebug("Sending initial state command: " + str(
#             initial_status_msg.path) + " on topic '" + server + INIT_TOPIC + "'")
#         init_pub = rospy.Publisher(server + INIT_TOPIC,
#                                    msgs.ContainerInitialStatusCmd, queue_size=1)
#         init_pub.publish(initial_status_msg)
#         #print(msgs.ContainerInitialStatusCmd)
#
#         start_time = rospy.Time.now()
#
#         # Block until we get a new state back
#         if timeout is not None:
#             while rospy.Time.now() - start_time < timeout:
#                 # Send the initial state command
#                 init_pub.publish(initial_status_msg)
#                 print(msg_response.path)
#                 # Filter messages that are from other containers
#                 if msg_response.path == path:
#                     # Check if the heartbeat came back to match
#                     state_match = all([s in msg_response.current for s in initial])
#                     print(state_match)
#                     rospy.logdebug("STATE MATCH: " + str(state_match))
#
#                     if state_match:
#                         return True
#                 rospy.sleep(0.3)
#             return False


class Proxy():
    """Pyhsm Container Introspection proxy.
    This class is used as a container for introspection and debugging.
    """

    def __init__(self, server_name, container, path, update_rate=rospy.Duration(2.0)):
        """Constructor for tree-wide data structure.
        """
        self._path = path
        self._container = container
        self._update_rate = update_rate
        self._structure_pub_lock = threading.Lock() # Needed, otherwise the structure won't be published
        
#        # Advertise init service
#        self._init_cmd = rospy.Subscriber(
#            server_name + INIT_TOPIC,
#            msgs.ContainerInitialStatusCmd,
#            self._init_cmd_cb)

        # Advertise structure publisher
        self._structure_pub = rospy.Publisher(
            name=server_name + STRUCTURE_TOPIC,
            data_class=msgs.HsmStructure,
            queue_size=1)

        # Advertise status publisher
        self._status_pub = rospy.Publisher(
            name=server_name + STATUS_TOPIC,
            data_class=msgs.HsmStatus,
            queue_size=1)
        rospy.sleep(1) # Otherwise the message gets lost and won't be published
        
        self._keep_running = False
        
    def start(self):
        self._keep_running = True
        
    def stop(self):
        self._keep_running = False
            
            
    def _publish_structure(self):
        '''Publish the structure of the whole state machine'''
        
        def _get_all_states(self):
            '''Returns list with all states of the state machine'''
            all_states = []
            # Only for the first iteration, add the root state to the list of 
            #   states which are examined for substates
            i = 1
            if i == 1:
                test_for_substates = [self._container]
            # Search for substates until we reach the leaf-states of the sm
            while test_for_substates:
                #
                for state in test_for_substates:
                    TestedState =[state]
                    # Only if the currently tested state has substates
                    #   add them to the test_for_substates list for further examination
                    if isinstance (state, Container):
                        test_for_substates.extend(state.states)
                    # Add the currently tested state to the list of all states
                    all_states.extend(TestedState)
                    # Remove the currently tested state from the list for further examination
                    test_for_substates.remove(state)
                i += 1   
            return all_states
            
        def _create_transition_msg(state):
            '''Puts the transition message together'''
            event = []
            target = []
            condition = []
            before = []
            action = []
            after = []
            # Get the information in the defaultdict which contains all transition related information
            for k,transition in state._transitions.items():
                for t in transition:
                    event.append(str(k))
                    target.append(str(t['to_state'].name))
                    #TODO: .__name__ ergibt _nop wenn leer, aber nichts wenn voll?
                    condition.append(str(t['condition'].__name__))
                    before.append(str(t['before'].__name__))
                    action.append(str(t['action'].__name__))
                    after.append(str(t['after'].__name__))
            # Put the transition message together
            transition_msg = msgs.Transition(
                event,
                target,
                condition,
                before,
                action,
                after)
            return transition_msg
            
        def _create_state_msg(state):
            '''Puts the state message together'''
            path = state._get_path()
            initial = str()
            if isinstance (state, Container):
                # It's only one initial state per container, but we have to get it out of its list:
                for i in state.get_initial_states():
                    initial = str(i.name)
            # Get the transition message for this state      
            transition_msg = _create_transition_msg(state)
            # Put the state message together
            state_msg.append(msgs.HsmState(
                path,
                initial,
                transition_msg))
            return state_msg
    
        # Create the msgs.HsmStructure
        with self._structure_pub_lock:
            #Iterate over all states in the state machine
            state_msg = []
            for every_state in _get_all_states(self):
                state_msg = _create_state_msg(every_state)
            # Construct the structure_msg        
            prefix = 'Prefix'
            structure_msg = msgs.HsmStructure(prefix, state_msg)
        try:
            # Publish message
            self._structure_pub.publish(structure_msg)
        except:
            if not rospy.is_shutdown():
                rospy.logerr("Publishing PYHSM introspection structure message failed.")


    def _publish_status(self, server_name, info_str=""):
        """Publish current state of the state machine."""
        # Construct status message
        # Current state of the sm is the active state in the deepest level of the sm
        leaf_state = self._container.leaf_state
        current = leaf_state.name
        path = leaf_state._get_path()
            # Construct status message
        status_msg = msgs.HsmStatus(
            path,
            current)
            
        try:
            self._status_pub.publish(status_msg)
            #print('Status_msg: ' + str(status_msg))
        except:
            if not rospy.is_shutdown():
                rospy.logerr("Publishing PYHSM introspection status message failed.")
            

#    def _init_cmd_cb(self, msg):
#        """Initialize a tree's state and userdata."""
#        initial = msg.initial
#
#        # Check if this init message is directed at this path
#        rospy.logdebug('Received init message for path: ' + msg.path + ' to ' + str(initial))
#        if msg.path == self._path:
#            if all(s in self._container.get_children() for s in initial):
#                rospy.logdebug("Setting initial state in pyhsm path: '" + self._path + "' to '" + str(
#                    initial))
#
#                # Set the initial state
#                self._container.set_initial_state(
#                    initial)
#                # Publish initial state
#                self._publish_status("REMOTE_INIT")
#            else:
#                rospy.logerr("Attempting to set initial state in container '" + self._path + "' to '") #+ str(
#                   # initial_states) + "', but this container only has states: " + str(self._container.get_children()))


class IntrospectionServer():
    """Server for providing introspection and control for pyhsm."""

    def __init__(self, server_name, machine, path):
        """Traverse the pyhsm tree starting at root, and construct an introspection
        proxy for getting and setting debug state."""
        # Store args
        self._server_name = server_name
        self._machine = machine
        self._path = path
        
        self._proxy = self.construct(self._server_name, self._machine, self._path)
        # Publish the structure of the state machine at the beginning
        self._proxy._publish_structure()
        # Create thread to constantly check for changes in the sm
        self._check_for_changes_thread = threading.Thread(name=server_name + ':check_for_changes', target=self._check_for_changes)
        self._keep_running = False


    def _check_for_changes(self):
        '''Constantly check for status changes and if a change occured, 
        publish a Status msg'''
        self.msg_path = str() # Initialize the msg_path string
        
        def _changes_cb(msg):
            #old_path = self.msg_path # Just for the print command
            self.msg_path = msg.path # Update msg_path string which is used for detecting changes
            #print('Old msg_path: ' + old_path + '\n' + 'New msg_path: ' + self.msg_path)
        # Create Subscriber to Status Topic to be able to register any descrepancies
        # between the latest published Status and the real status of the sm
        state_changed = rospy.Subscriber('pyhsm_introspection' + STATUS_TOPIC, msgs.HsmStatus, callback=_changes_cb)
        rospy.sleep(1) # Needed to register the Subscriber bevore something is published
        
        # Constantly check if the active state in the sm has changed
        # If so, publish the current status
        while not rospy.is_shutdown() and self._keep_running:
            real_path = self._machine.leaf_state._get_path()
            # TODO: Error l. 311: AttributeError: 'NoneType' object has no attribute '_get_path'
            if self.msg_path != real_path:
                self._proxy._publish_status(self._server_name, 'HEARTBEAT')
                rospy.sleep(1)


    def start(self):
        # get informed about transitions
        self._machine.register_transition_cb(self._transition_cb, self._proxy)
        # handle __TRANSITION__ events
        self._machine.add_handler('__TRANSITION__', self._transition_handler)

        self._transition_cmd = rospy.Subscriber(
            self._server_name + TRANSITION_TOPIC,
            String, self._transition_cmd_cb)
        self._transition_cmd = rospy.Subscriber(
            self._server_name + EVENT_TOPIC,
            String, self._event_trigger_cb)
        self._keep_running = True
        #Start thread which constantly checks for status changes
        self._check_for_changes_thread.start()


    def stop(self):
        self._proxy.stop()
        self._keep_running = False


    def construct(self, server_name, state, path):
        """Construct Proxy for sm."""
        # Construct a new proxy
        proxy = Proxy(server_name, state, path)
        if path == '/':
            path = ''
        proxy.start()
        #print('New Proxy created!')
        return proxy

    #TODO: Do we need an equivalent of this for one single proxy or is proxy.stop enough?
#    def clear(self):
#        """Clear all proxies in this server."""
#        self._proxies = []
    
    #TODO: Does this work, regarding the changes at the way a Status is published?
    def _transition_cb(self, from_state, to_state, proxy):
        self._proxy._publish_status()

    def _transition_cmd_cb(self, msg):
        to_state = self._machine
        try:
            for k in msg.data.split('/')[1:]:
                to_state = to_state[k]
        except:
            rospy.logerr('Unknown state %s' % msg.data)
            return

        # dispatch an event to trigger the transition (don't call _transition_to() directly!)
        self._machine.dispatch(hsm.Event('__TRANSITION__', to_state=to_state))

    def _event_trigger_cb(self, msg):
        self._machine.dispatch(msg.data)

    def _transition_handler(self, event):
        self._machine._transition_to(event.userdata['to_state'], event=None)