#!/usr/bin/env python3
# -*- coding:utf8 -*-
import tornado.ioloop
import tornado.web
import tornado.httpserver
import logging
from tornado.escape import json_decode, json_encode, utf8
from ros_car_control import RosCarControl
from ros_car_sensors import RosCarSensors

logger = logging.getLogger('mylogger')
logger.setLevel(logging.DEBUG)

car_control_speed = 0

class RosCarControlAPI(object):
    def __init__(self):
        global control,sensors
        control = RosCarControl()
        sensors = RosCarSensors()

    def make_app(self):
        return tornado.web.Application([
            (r"/car_control", ControlAPI),
            (r"/car_sensor", SensorAPI)
        ])

    def start_control(self):
        app = self.make_app()
        sockets = tornado.netutil.bind_sockets(10031)
        http_server = tornado.httpserver.HTTPServer(app)
        http_server.add_sockets(sockets)
        print("Server Start Ok.....")
        
        tornado.ioloop.IOLoop.instance().start()

class ControlAPI(tornado.web.RequestHandler):
    def get(self, *args, **kwargs):
        global car_control_speed
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "x-requested-with")
        self.set_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.set_header('Content-Type', 'application/json')
        car_control_type = self.get_query_argument("type",default="",strip=True)
        car_control_cmd = self.get_query_argument("cmd",default="",strip=True)
        car_control_speed = self.get_query_argument("speed",default="",strip=True)
        car_control_angular = self.get_query_argument("angular",default="",strip=True)

        car_result = control.car_control(car_control_type, car_control_cmd, car_control_speed, car_control_angular)
        result = {
            "code": 200,
            "msg": car_result   
        }
        self.write(json_encode(result))
    def post(self):
        global car_control_speed
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "x-requested-with")
        self.set_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.set_header('Content-Type', 'application/json')
        car_control_type = self.get_body_argument("type",default="",strip=True)
        car_control_cmd = self.get_body_argument("cmd",default="",strip=True)
        car_control_speed = self.get_body_argument("speed",default="",strip=True)
        car_control_angular = self.get_body_argument("angular",default="",strip=True)
        car_result = control.car_control(car_control_type, car_control_cmd, car_control_speed, car_control_angular)
        result = {
            "code": 200,
            "msg": car_result
        }
        self.write(json_encode(result))

class SensorAPI(tornado.web.RequestHandler):

    def get(self, *args, **kwargs):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "x-requested-with")
        self.set_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.set_header('Content-Type', 'application/json')

        sensor_info = sensors.get_sensor_info(car_control_speed)
        result = {
            "code": 200,
            "msg": sensor_info
        }
        self.write(json_encode(result))

if __name__=='__main__':
    car = RosCarControlAPI()
    car.start_control()


    
    
