import sys

from myrobut_interfaces.srv import PerformMove
import rclpy
from rclpy.node import Node

import sdl2.ext
import time,math,copy,os

#Consts:
#quarter = math.pi/2
higlight_threshold = 0.05 #with the_position, in rads...
wheel_step = float(1/5)  #n steps on each side of the wheel

#slowestSpeed = 0.22  #trial and error
slowestSpeed = 0.4  #trial and error
fastestSpeed = 0.9499999999999998  #dont supply the full 12 V to the motors or it will deprive the RPi
midRangeSpeed = 0.6

wheel_step_delay = float(1/3)  #how long to hold left/right before it goes to the next wheel step, in seconds...
main_delay = float(1/1000)  #1kHZ, main programme loop timing, check for new events this often...
max_execution_time = 0.3

steadyStreamInterval = 0.3


#Global Variables:
#wheel position -1.0 is full left, 1.0 is full right:
current_wheel_position = 0.0
#Power: True is GO (Write Staged Values), and False is STOP
power = False
#Reverse (bool) (Direction):
reverse = False
#Orientation of the robut:
upside_down = True
#calculated from steering wheel position only:
staged_left = 0.0
staged_right = 0.0
#last values actually written out to the motor driver:
actual_left = 0.0
actual_right = 0.0
#initial keydown timestamps (None means key is not held down at the moment...):
keydown_left_timestamp = None
keydown_right_timestamp = None
keydown_up_timestamp = None
keydown_down_timestamp = None
#XXX:pysdl2 stuff:
window = None
RESOURCES = None
factory = None
spriterenderer = None
surfacerenderer = None
dash_label_sprite = None
the_sprite_factory = None
rectangles_map = None  # map of: (theta value) -> [sprite object]


class PerformMoveClientAsync(Node):

    def __init__(self):
        super().__init__('perform_move_client_async')
        self.cli = self.create_client(PerformMove, 'perform_move')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PerformMove.Request()

    def send_request(self,a_req_number,a_drive_left,a_drive_right,a_max_execution_time):
        self.req.req_number = a_req_number
        self.req.drive_left = a_drive_left
        self.req.drive_right = a_drive_right
        self.req.max_execution_time = a_max_execution_time
        self.future = self.cli.call_async(self.req)

#### END class PerformMoveClientAsync ####



def loadSprites():
    global factory,the_sprite_factory,dash_label_sprite
    the_sprite_factory = {
        '0': lambda: factory.from_image(RESOURCES.get_path("0.png")),
        '1': lambda: factory.from_image(RESOURCES.get_path("1.png")),
        '2': lambda: factory.from_image(RESOURCES.get_path("2.png")),
        '3': lambda: factory.from_image(RESOURCES.get_path("3.png")),
        '4': lambda: factory.from_image(RESOURCES.get_path("4.png")),
        '5': lambda: factory.from_image(RESOURCES.get_path("5.png")),
        '6': lambda: factory.from_image(RESOURCES.get_path("6.png")),
        '7': lambda: factory.from_image(RESOURCES.get_path("7.png")),
        '8': lambda: factory.from_image(RESOURCES.get_path("8.png")),
        '9': lambda: factory.from_image(RESOURCES.get_path("9.png")),
        '-': lambda: factory.from_image(RESOURCES.get_path("minus.png")),
        '.': lambda: factory.from_image(RESOURCES.get_path("dot.png"))}
    dash_label_sprite = factory.from_image(RESOURCES.get_path("myrcdash.png"))


def make_unhilighted_wheel_sprite(x,y):
    global factory,RESOURCES
    the_new_sprite = factory.from_image(RESOURCES.get_path("red25x25.png"))
    the_new_sprite.x = x
    the_new_sprite.y =y
    return the_new_sprite


def make_hilighted_wheel_sprite(x,y):
    global factory,RESOURCES
    the_new_sprite = factory.from_image(RESOURCES.get_path("white25x25.png"))
    the_new_sprite.x = x
    the_new_sprite.y =y
    return the_new_sprite


def make_spriteset(figure_float, left, top):
    global the_sprite_factory
    retval=[]
    x=left; y=top
    for i in '%.02f' % (figure_float):
       a_sprite = the_sprite_factory[i]()
       a_sprite.x = x
       a_sprite.y = y
       x = x + a_sprite.size[0]
       #spriterenderer.render(a_sprite)  #https://github.com/marcusva/py-sdl2/issues/91 --> "To prevent flickering this requires all sprites that are created in their own functions to be returned back to the main loop and rendered at once in a single call to SpriteRender::render in a list. "
       retval.append(a_sprite)
    return retval


#returns list of sprite objects:
def drawDashboard():
    left_margin = 85
    staged_left_spriteset = make_spriteset(staged_left, left_margin, 65)
    staged_right_spriteset = make_spriteset(staged_right, left_margin, 125)
    actual_left_spriteset = make_spriteset(actual_left, left_margin, 270)
    actual_right_spriteset = make_spriteset(actual_right, left_margin, 335)
    numerical_figure_spriteset = make_spriteset(current_wheel_position, left_margin, 400)
    return [dash_label_sprite]+staged_left_spriteset+staged_right_spriteset+actual_left_spriteset+actual_right_spriteset+numerical_figure_spriteset


#returns dict like:
#   { (theta value):  [sprite object],
#     ... }
def generate_rectangles():
   global current_wheel_position,rectangles_map
   rectangles_map ={}
   circle_left = 250
   circle_right = 490
   circle_bottom = 250
   r=circle_bottom
   twor=2*r;
   #The factors to scale out to actual, graphical X-Y components:
   x_factor = 1
   y_factor = 1

   #the_position_as_theta=math.pi+math.pi*((current_wheel_position + 1.0)/2)
   #print ('r is %d;' %(r))
   for theta in [math.pi+ math.pi*(x/twor) for x in range(0,twor+1)]:
     y=int(round(circle_bottom+r*math.sin(theta)))
     x=int(round(circle_left+r*math.cos(theta)))
     #print('y is %d, x is %d; ' % (y,x))
     some_sprite = make_unhilighted_wheel_sprite(x * x_factor + circle_left, y * y_factor)
     rectangles_map[theta] = some_sprite


#update the sprite object(s) for the appropriate 'hilighted' sterring wheel segment(s)
#returns updated `rectangles_map`:
def updateSteeringWheelSprites():
   global rectangles_map
   the_position_as_theta=math.pi+math.pi*((current_wheel_position + 1.0)/2)
   for x_theta in rectangles_map.keys():
     if abs(x_theta-the_position_as_theta) < higlight_threshold:
       #TODO: check if is this a memory leak?
       rectangles_map[x_theta] = make_hilighted_wheel_sprite(rectangles_map[x_theta].x, rectangles_map[x_theta].y)
     else:
       #restore to unhilighted:
       rectangles_map[x_theta] = make_unhilighted_wheel_sprite(rectangles_map[x_theta].x, rectangles_map[x_theta].y)


#TURNING IMPLEMENTION #1 -Use negative component of 'current_wheel_position' to calculate staged_left:
#Use positive component of 'current_wheel_position' to calculate staged_right:
def calculate_staged_values(base_speed):
    global staged_left,staged_right,current_wheel_position
    zero_threshold = 0.001  #https://docs.python.org/3/tutorial/floatingpoint.html
    #print('current wheel position is %f' % (current_wheel_position))
    if current_wheel_position > -zero_threshold and current_wheel_position < zero_threshold:
        staged_left = base_speed
        staged_right = base_speed
        return
    if current_wheel_position < 0:
        negative_component = abs(current_wheel_position)
        positive_component = 1 - abs(current_wheel_position)
    elif current_wheel_position > 0:
        negative_component = 1- current_wheel_position
        positive_component = current_wheel_position
    #print('negative_component is %f, positive_component is %f' %(negative_component,positive_component))
    staged_left = base_speed * negative_component
    staged_right = base_speed * positive_component

#TURNING IMPLEMENTATION #2 - Use `current_wheel_position` to reduce the speed of the "other" channel ONLY:
def calculate_staged_values2(base_speed):
    global staged_left,staged_right,current_wheel_position
    zero_threshold = 0.001  #https://docs.python.org/3/tutorial/floatingpoint.html
    #print('current wheel position is %f' % (current_wheel_position))
    if current_wheel_position > -zero_threshold and current_wheel_position < zero_threshold:
        staged_left = base_speed
        staged_right = base_speed
        return
    if (not upside_down and current_wheel_position < 0) or (upside_down and current_wheel_position > 0):
        staged_left = base_speed
        staged_right = (1 - abs(current_wheel_position)) * base_speed 
    elif (not upside_down and current_wheel_position > 0) or (upside_down and current_wheel_position < 0):
        staged_left = (1 - abs(current_wheel_position)) * base_speed 
        staged_right = base_speed


#return True if changed...:
#Updates keydown_up_timestamp,keydown_down_timestamp
#Updates power and reverse...:
def processUpDownEvents(someEvents):
    global power,keydown_up_timestamp,keydown_down_timestamp,reverse
    for event in someEvents:
      if event.type == sdl2.SDL_KEYDOWN:
        if event.key.keysym.sym == sdl2.SDLK_UP and keydown_up_timestamp == None:
            keydown_up_timestamp = time.time()
            power = True  
            reverse = False  #Forward Direction
            return True
        elif event.key.keysym.sym == sdl2.SDLK_DOWN and keydown_down_timestamp == None:
            #TODO: gear down???
            keydown_down_timestamp = time.time()
            power = True
            reverse = True #<-- Backwards Direction
            return True
      elif event.type == sdl2.SDL_KEYUP:
        if event.key.keysym.sym == sdl2.SDLK_UP:
            #TODO: gear down instead of zeroing?
            keydown_up_timestamp = None
            power = False   #Stop
            return True
        if event.key.keysym.sym == sdl2.SDLK_DOWN:
            #TODO: gear down instead of zeroing?
            keydown_down_timestamp = None
            power = False   #Stop
            return True
    return False

   

#Updates keydown_{left,right}_timestamp
#Update current_wheel_position...:
def processLeftRightEvents(someEvents):
    global keydown_left_timestamp,keydown_right_timestamp,current_wheel_position
    for event in someEvents:
        if event.type == sdl2.SDL_KEYDOWN:
            if event.key.keysym.sym == sdl2.SDLK_LEFT and keydown_left_timestamp == None:
                current_wheel_position = max(-1.0,current_wheel_position - wheel_step)
                keydown_left_timestamp = time.time()
                return True
            elif event.key.keysym.sym == sdl2.SDLK_RIGHT and keydown_right_timestamp == None:
                current_wheel_position = min(1.0,current_wheel_position + wheel_step)
                keydown_right_timestamp = time.time()
                return True
        elif event.type == sdl2.SDL_KEYUP:
            if event.key.keysym.sym == sdl2.SDLK_LEFT:
                keydown_left_timestamp = None
                return True
            elif event.key.keysym.sym == sdl2.SDLK_RIGHT:
                keydown_right_timestamp = None
                return True
    return False

#returns True if `current_wheel_position` was updated, False otherwise...
def processHeldDownLeftRightKeys():
    global keydown_left_timestamp,keydown_right_timestamp,current_wheel_position,wheel_step
    if keydown_left_timestamp != None and keydown_right_timestamp == None:
        if time.time() - keydown_left_timestamp >= wheel_step_delay:
           #print('adjust current_wheel_position to the left!')
           current_wheel_position = max(-1.0,current_wheel_position - wheel_step)
           keydown_left_timestamp = time.time()  #update LEFT keydown timestamp...
           return True
    elif keydown_right_timestamp != None and keydown_left_timestamp == None:
        if time.time() - keydown_right_timestamp >= wheel_step_delay:
           #print('adjust current_wheel_position to the right!')
           current_wheel_position = min(1.0,current_wheel_position + wheel_step)
           keydown_right_timestamp = time.time()  #update RIGHT keydown timestamp...
           return True
    return False


def main(args=None):
    global window,RESOURCES,factory,dash_label_sprite,spriterenderer,surfacerenderer,rectangles_map,main_delay,actual_left,actual_right,staged_left,staged_right,reverse

    sdl2.ext.init()
    window = sdl2.ext.Window('py_myrobut client', size=(800,500))
    window.show()
    RESOURCES = sdl2.ext.Resources(os.path.dirname(__file__)+'/../../../../share/ament_index/resource_index/packages')
    factory = sdl2.ext.SpriteFactory(sdl2.ext.SOFTWARE)
    spriterenderer = factory.create_sprite_render_system(window)
    surfacerenderer = sdl2.ext.Renderer(window.get_surface())
    surfacerenderer.color = sdl2.ext.Color(0,0,0)

    loadSprites()
    generate_rectangles()
  
    rclpy.init(args=args)

    perform_move_client = PerformMoveClientAsync()

    valuesChangedFlag = True  #<-- for 1st loop
    reqNumber = 0
    lastValuesSent = 0.0
    while True:
        events = sdl2.ext.get_events()
        powerChangeEvent = False

        if processLeftRightEvents(events):
            valuesChangedFlag = True

        if processUpDownEvents(events):
            powerChangeEvent = True
            valuesChangedFlag = True
  
        if processHeldDownLeftRightKeys():
            valuesChangedFlag = True
        #else: #TODO: <-- push the steering wheel back to `centre` position if neeither LEFT nor RIGHT keys being held down....

        if valuesChangedFlag:    #redraw:
            calculate_staged_values2(midRangeSpeed)
            #Calculate actual values:
            my_direction = 1.0 if reverse else -1.0
            if power:
                actual_left = my_direction * staged_left
                actual_right = my_direction * staged_right
            else:
                actual_left = 0.0
                actual_right = 0.0
            surfacerenderer.clear()
            dashboard_sprites = drawDashboard()
            updateSteeringWheelSprites()
            spriterenderer.render(dashboard_sprites + list(rectangles_map.values()))
            surfacerenderer.present()
            #NOTE: window.refresh() does not appear needed!

        if ((power or powerChangeEvent) and valuesChangedFlag)  or (time.time() - lastValuesSent > steadyStreamInterval):    #TODO: OR IF TIME TO EMIT A "STEADY STREAM" EVENT??
            lastValuesSent=time.time()
            reqNumber = reqNumber + 1
            perform_move_client.send_request(reqNumber, actual_left,actual_right,max_execution_time)  #TODO: make new constant for this one...
            print('[%f] #%d - Sending event!' % (time.time(), reqNumber))
            rclpy.spin_once(perform_move_client)
            print('[%f] #%d - Sent event!\n' % (time.time(), reqNumber))
            #TO WAIT:
            #while rclpy.ok():
            ####DELETEME:    rclpy.spin_once(perform_move_client)
            #    if perform_move_client.future.done():
            #        try:
            #            response = perform_move_client.future.result()
            #        except Exception as e:
            #            perform_move_client.get_logger().info(
            #                'Service call failed %r' % (e,))
            #        else:
            #            perform_move_client.get_logger().info('OK')
            #        break
        time.sleep(main_delay)
        valuesChangedFlag = False

    #TODO: clean-up:
    #print ('file is %s' % (__file__))
    #print ('dash_label_sprite is:%s'%(str( dash_label_sprite)))


    while rclpy.ok():
        rclpy.spin_once(perform_move_client)
        if perform_move_client.future.done():
            try:
                response = perform_move_client.future.result()
            except Exception as e:
                perform_move_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                perform_move_client.get_logger().info('OK')
            break

    perform_move_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
