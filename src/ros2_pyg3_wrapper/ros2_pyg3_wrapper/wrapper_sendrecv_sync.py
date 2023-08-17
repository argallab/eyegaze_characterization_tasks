"""
Python wrapper for Tobii Pro Glasses 3 API
Contains PyGlasses3 class which wraps the API into convenient 
function calls 

Some useful references:
https://github.com/ricardolsmendes/websockets-asyncio/blob/main/document_inspector.py
https://stackoverflow.com/questions/67734115/how-to-use-multithreading-with-websockets

"""

import websockets
import asyncio
import json
import cv2
import logging
import threading
import sys
import base64

# sys.path.insert(0, '/home/eyetrack_ws/src/')
from PyGlasses3.utilities import MsgID
from PyGlasses3.sender import Sender
from PyGlasses3.receiver import Receiver


class PyGlasses3(object):
    def __init__(self, glasses_addr="[fe80::76fe:48ff:fe6b:faea]", ipv6=False):
        self.glasses_addr = glasses_addr
        
        ## initialize Sender and Receiver objects
        self.sender = Sender()
        self.receiver = Receiver()

        if ipv6:
            self.__ws_url = "ws://[{}]/websocket".format(self.glasses_addr + "%eno1")
            self.__rtsp_url = "rtsp://[{}]:8554/live/all".format(self.glasses_addr + "%eno1")
        else:
            self.__ws_url = "ws://{}/websocket".format(self.glasses_addr)
            self.__rtsp_url = "rtsp://{}:8554/live/all".format(self.glasses_addr)
        print("glasses url is: {}".format(self.__ws_url))

    def client(self):
        """ Sets up websocket client. 

        Runs __connect.
        This function should be run on a separate thread from the main function
        Main function should call class methods that append dicts to sender.msgs 
        for communicating with the websocket. 
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.__connect())
        loop.close()

    async def __connect(self):
        """ Function that connects to the websocket and sends and receives messages

        Set up websocket connection 
        Reestablish connection when connection fails
        Send and receive messages.
        """
        async for websocket in websockets.connect(self.__ws_url, subprotocols=["g3api"]):
            try:
                print("connected!")
                await self.__send_message(websocket)
                recv_msg = await self.__receive_message(websocket)
                self.receiver.msgs.append(json.loads(recv_msg))
                print(recv_msg)

            except websockets.ConnectionClosed:
                ## reestablish the websocket connection when it fails
                continue

    async def __receive_message(self, websocket):
        """ Receives a message from the websocket 
        """
        received_msg = await websocket.recv()
        return received_msg

    async def __send_message(self, websocket):
        """ Sends a mesage to the websocket

        If there are no messages to send, just pass
        """
        try:
            send_msg = self.sender.msgs[0]
            await websocket.send(json.dumps(send_msg))
            self.sender.msgs.pop(0)
        except:
            pass

