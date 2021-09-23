from myrobut_interfaces.srv import PerformMove

import rclpy
from rclpy.node import Node

import TB3
import time
import sys,traceback
import threading



class PerformMoveService(Node):

    def __init__(self,aTB):
        super().__init__('perform_move_service')
        self.srv = self.create_service(PerformMove, 'perform_move', self.perform_move_callback)
        self.lastPerformMoveCallTime = time.time()
        self.theTB = aTB

    def perform_move_callback(self, request, response):
        timeNow = time.time()    #<-- use "ros time" here??
        self.get_logger().info('[%f] Incoming request #%d: drive_left: %f drive_right: %f max_execution_time: %f' % (timeNow, request.req_number, request.drive_left, request.drive_right,request.max_execution_time))
        #Note: https://stackoverflow.com/questions/2291069/is-python-variable-assignment-atomic
        #"Simple assignment to simple variables is "atomic" AKA threadsafe"
        #So a lock may not be necessarry...
        self.lastPerformMoveCallTime = timeNow 
        if self.theTB != None:  #<--TODO: check if a performance issue?
            self.theTB.SetMotor1(request.drive_left)
            self.theTB.SetMotor2(request.drive_right)
        
        return PerformMove.Response()

maxTimeWithoutClientCall = 0.5
def safety_thread_function(perform_move_service_object):
    while True:
        if time.time() - perform_move_service_object.lastPerformMoveCallTime > maxTimeWithoutClientCall:
            print('[%f] SAFETY THREAD: MOTORS OFF DUE TO MAX_TIME_WITHOUT_CLIENT_CALL EXCEEDED !!!' % (time.time()))
            if perform_move_service_object.theTB != None:
                perform_move_service_object.theTB.MotorsOff()
        time.sleep(maxTimeWithoutClientCall)

def main(args=None):
    try:
        TB = TB3.ThunderBorg()
        TB.Init()
        if not TB.foundChip:
            boards = TB.ScanForThunderBorg()
            if len(boards) == 0:
                print ('No ThunderBorg found, check you are attached :)')
            else:
                print ('No ThunderBorg at address %02X, but we did find boards:' % (TB.i2cAddress))
                for board in boards:
                    print ('    %02X (%d)' % (board, board))
                print ('If you need to change the I²C address change the setup line so it is correct, e.g.')
                print ('TB.i2cAddress = 0x%02X' % (boards[0]))
            sys.exit()
        
        TB.SetCommsFailsafe(False)             # Disable the communications failsafe
    except: 
        print ('Error Loading ThunderBorg: %s ' % (str(sys.exc_info())))
        traceback.print_exc()
        print ('continuing in TEST MODE ONLY!')
        TB=None

    rclpy.init(args=args)

    perform_move_service = PerformMoveService(TB)
    safetyThread = threading.Thread(target=safety_thread_function, args=(perform_move_service,))
    safetyThread.daemon = True  #So that the Safety Thread shouldnt block programme exit...
    safetyThread.start()
    #NOTE: Service calls/requests are BLOCKING:
    rclpy.spin(perform_move_service)
    

    rclpy.shutdown()


if __name__ == '__main__':
    main()
