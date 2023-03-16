import sys,re
import threading


from myrobut_interfaces.srv import PerformMove
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,ByteMultiArray,UInt8MultiArray

import sdl2.ext
import time,math,copy,os

from PIL import Image
import io
from sdl2 import surface,endian
import urllib
from sdl2 import sdlimage
import ctypes



#Consts:
#slowestSpeed = 0.22  #trial and error
#slowestSpeed = 0.4  #trial and error
slowestSpeed = 0.3  #trial and error
fastestSpeed = 0.9499999999999998  #dont supply the full 12 V to the motors or it will deprive the RPi
main_delay = float(1/1000)  #1kHZ, main programme loop timing, check for new events this often...
desired_fps = 24
#midRangeSpeed = 0.6
width = int (1440/2)
height= int(720/2)

max_execution_time = 0.3
steadyStreamInterval = 0.3


#Global Variables:
#Power: True is GO (Write Staged Values), and False is STOP
power = False
#Orientation of the robut:
upside_down = True
#values actually written out to the motor driver:
#XXX:pysdl2 stuff:
from sdl2 import SDL_FINGERDOWN,SDL_FINGERUP,SDL_FINGERMOTION
window = None
RESOURCES = None
factory = None
spriterenderer = None
lineTexture = None
lineSrcRect = None
hlineDstRect = None
vlineDstRect = None


class PerformMoveClientAsync(Node):

    def __init__(self):
        super().__init__('perform_move_client_async')
        self.cli = self.create_client(PerformMove, 'perform_move')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PerformMove.Request()
        #self.subscription = self.create_subscription(
        #  UInt8MultiArray,
        #  'topic',
        #  self.listener_callback,
        #  1)
        #self.subscription # prevent unused warning
 
    def listener_callback(self ,msg):  #XXX - not used
        global spriterenderer
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #the_bytes=open('image.jpg','rb').read()
        the_bytes=msg.data
        #print (str(type(the_bytes)))
        #print ("len="+str(len(the_bytes)))
        #print (str(the_bytes[0]))
        #print ("len0="+str(len(the_bytes[0])))
        sprite=makeSpriteFromJpegBytes(the_bytes)
        #sprite.size = (1440, 720)
        #sprite.scale = (2.0,2.0)
        
        spriterenderer.render(sprite)

    def send_request(self,a_req_number,a_drive_left,a_drive_right,a_max_execution_time):
        self.req.req_number = a_req_number
        self.req.drive_left = a_drive_left
        self.req.drive_right = a_drive_right
        self.req.max_execution_time = a_max_execution_time
        self.future = self.cli.call_async(self.req)

#### END class PerformMoveClientAsync ####




def makeSpriteFromJpegBytes(the_bytes):  #PIL-Based 
    image=Image.open(io.BytesIO(the_bytes))
    pxbuf = image.tobytes()
    width, height = image.size
    mode = image.mode
    #print ('width=%d height=%d mode=%s' %(width,height,mode))
    rmask = gmask = bmask = amask = 0
    #mode='RGB':
    #depth=24 pitch=1536 rmask=ff gmask=ff00 bmask=ff0000 amask=0
    rmask = 0xFF
    gmask = 0xFF00
    bmask = 0xFF0000
    depth = 24
    pitch = width * 3
    #print('depth=%d pitch=%d rmask=%x gmask=%x bmask=%x amask=%x' % (depth, pitch, rmask,gmask, bmask, amask))
    imgsurface = surface.SDL_CreateRGBSurfaceFrom(pxbuf, width, height,
                                                  depth, pitch, rmask,
                                                  gmask, bmask, amask)
    imgsurface = imgsurface.contents
    # the pixel buffer must not be freed for the lifetime of the surface
    imgsurface._pxbuf = pxbuf
    sprite = factory.from_surface(imgsurface)
    return sprite


def makeSurfaceFromJpegBytesNew(some_bytes):  #SDL2_Image.so-Based
    src=sdl2.SDL_RWFromMem(some_bytes, len(some_bytes))
    imgsurface = sdlimage.IMG_Load_RW(src, 1)   #1 means "close/free the SDL_RWops before returning"

    if not imgsurface:
        # An error occured - if we do not try PIL, break out now
        if not _HASPIL or enforce == "SDL":
            raise SDLError(sdlimage.IMG_GetError())
    else:
        imgsurface = imgsurface.contents
    
    return imgsurface


desired_fps=24
duration_next_img = 1.0/desired_fps
def video_thread_function(THE_IP,RENDERER):
  global duration_next_img,window,lineTexture,lineSrcRect,hlineDstRect,vlineDstRectrenderer
  while True:
      #do this on a seperate thread in case it blocks!
      
      #if time.time() >= next_img_time:
      with urllib.request.urlopen('http://%s/html/cam_pic.php' % (THE_IP)) as response: #TODO: connection timeouts,etc.
      
        the_bytes = response.read()
      surface=makeSurfaceFromJpegBytesNew(the_bytes)
      sprite=factory.from_surface(surface)
      sprite.x = 104 # (width-imgwidth)/2
      sprite.y = 36 # ...
      RENDERER.render(sprite)
      sdl2.SDL_RenderCopy(renderer.renderer, lineTexture, lineSrcRect, hlineDstRect)
      sdl2.SDL_RenderCopy(renderer.renderer, lineTexture, lineSrcRect, vlineDstRect)
      #window.refresh()

      #next_img_time = time.time() + duration_next_img
      time.sleep(duration_next_img)
      sdl2.surface.SDL_FreeSurface( surface )
      

def main(args=None):
    global window,RESOURCES,factory,dash_label_sprite,spriterenderer,surfacerenderer,rectangles_map,main_delay,actual_left,actual_right,staged_left,staged_right,reverse,power,lineTexture,lineSrcRect,hlineDstRect,vlineDstRect,renderer

    sdl2.ext.init()
    window = sdl2.ext.Window('py_myrobut client', size=(width, height))  #max resolution of sideways screen at swaym scale factor 2 ...
    window.show()
    RESOURCES = sdl2.ext.Resources(os.path.dirname(__file__)+'/../../../../share/ament_index/resource_index/packages')
    #factory = sdl2.ext.SpriteFactory(sdl2.ext.SOFTWARE)  #TODO: use hardware acceleration?
    renderer = sdl2.ext.Renderer(window)
    factory = sdl2.ext.SpriteFactory(sdl2.ext.TEXTURE, renderer=renderer)
    spriterenderer = factory.create_sprite_render_system(window)  #TODO: figure out of sprites and lines can co-exist?
    sdlimage.IMG_Init(sdlimage.IMG_INIT_JPG) 
    #surfacerenderer = sdl2.ext.Renderer(window.get_surface())
    #surfacerenderer.color = sdl2.ext.Color(0,0,0)

    #Too much flickering if using sdl2.ext.TEXTURE-mode:
    ##draw a horizontal line:
    #whiteColor = sdl2.ext.Color(255,255,255)
    #x1, x2 = 0, width
    #y     = int(height/2)
    #sdl2.ext.line(window.get_surface(), whiteColor, (x1, y, x2, y))
    ##draw a vertical line:
    #x = int (width/2)
    #y1, y2 = 0, height
    #sdl2.ext.line(window.get_surface(), whiteColor, (x, y1, x, y2))

    #CreateTexture-based line drawing:
    lineTexture = sdl2.SDL_CreateTexture(renderer.renderer, sdl2.SDL_PIXELFORMAT_ARGB8888, sdl2.SDL_TEXTUREACCESS_STATIC, 1, 1)
    linePixels = bytearray([0xff]*3)
    pointer = (ctypes.c_char*len(linePixels)).from_buffer(linePixels)
    sdl2.SDL_UpdateTexture(lineTexture, None, pointer, 3)

    lineSrcRect=sdl2.SDL_Rect()
    lineSrcRect.x=0
    lineSrcRect.y=0
    lineSrcRect.w=1
    lineSrcRect.h=1

    hlineDstRect=sdl2.SDL_Rect()
    hlineDstRect.x=0
    hlineDstRect.y=int(height/2)
    hlineDstRect.w=width
    hlineDstRect.h=1

    vlineDstRect=sdl2.SDL_Rect()
    vlineDstRect.x=int(width/2)
    vlineDstRect.y=0
    vlineDstRect.w=1
    vlineDstRect.h=height

    #window.refresh()

    sdl2.SDL_RenderCopy(renderer.renderer, lineTexture, lineSrcRect, hlineDstRect)
    sdl2.SDL_RenderCopy(renderer.renderer, lineTexture, lineSrcRect, vlineDstRect)
    #draw a sprite:
    #loadSprites()
    #spriterenderer.render([dash_label_sprite])  #TODO: show something else here
  
    rclpy.init(args=args)

    perform_move_client = PerformMoveClientAsync()


    #Initial Values:
    running = True
    reqNumber = 0
    leftPowerValue = 0.0
    rightPowerValue = 0.0
    lastValuesSent = 0.0
    lastValuesChanged = 0.0
    valuesChangedFlag = True  #<-- for 1st loop
    duration_next_img = 1.0/desired_fps
    next_img_time = time.time() + duration_next_img
    
    THE_IP=re.findall('(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', open('DEFAULT_FASTRTPS_PROFILES.xml').read())[0]
    videoThread = threading.Thread(target=video_thread_function, args=(THE_IP,spriterenderer,))
    videoThread.daemon = True  #so as not to interfere with the ros2
    videoThread.start()

    while running:
      events = sdl2.ext.get_events()
    
      valuesChangedFlag = False
      for event in events:
                if event.type == sdl2.SDL_QUIT:
                    running = False
                    break
                if event.type == SDL_FINGERDOWN or event.type == SDL_FINGERMOTION:
                    if time.time()-lastValuesChanged > 0.1:  #TODO: add a constannt for thi sone...
                        #y is 0 at top of screen, 1 at bottom of screen
                        yPowerValue=2 * (event.tfinger.y - 0.5)
                        #Scale the value into the range (slowestSpeed, fastedSpeed)
                        scaledPowerValue = yPowerValue * (fastestSpeed - slowestSpeed)
                        thePowerValue = slowestSpeed + scaledPowerValue if scaledPowerValue > 0 else scaledPowerValue - slowestSpeed
                        if not upside_down: thePowerValue = -thePowerValue #because y is 0 attop of screen?
                        #print('%d: (%s,%s) thePowerValue=%f' % (event.type,event.tfinger.x,event.tfinger.y,thePowerValue))
                        #Calculate leftPowerValue and rightPowerValu:
                        leftPowerValue  = thePowerValue * ((1 - event.tfinger.x)*2 if event.tfinger.x > 0.5 else 1.0)
                        rightPowerValue = thePowerValue * ((event.tfinger.x*2 if event.tfinger.x < 0.5 else 1.0))
                        if upside_down: leftPowerValue, rightPowerValue = rightPowerValue, leftPowerValue
                        valuesChangedFlag = True
                        lastValuesChanged = time.time()
                if event.type == SDL_FINGERUP:
                    print('STOP POWER')
                    leftPowerValue=0.0
                    rightPowerValue=0.0
                    valuesChangedFlag = True
                    lastValuesChanged = time.time()
                    continue

      if valuesChangedFlag or (time.time() - lastValuesSent > steadyStreamInterval):  
          lastValuesSent=time.time()
          reqNumber = reqNumber + 1
          perform_move_client.send_request(reqNumber, leftPowerValue, rightPowerValue, max_execution_time) 
          #print('[%f] #%d - Sending event! ' % (time.time(), reqNumber))
          rclpy.spin_once(perform_move_client)
          #print('[%f] #%d - Sent event!\n' % (time.time(), reqNumber))

    sdl2.ext.quit()
    perform_move_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

