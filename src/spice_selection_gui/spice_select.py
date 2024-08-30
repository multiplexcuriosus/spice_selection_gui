import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from threading import Lock

from spice_selection_gui.srv._LocalizeColorMask import LocalizeColorMask,LocalizeColorMaskRequest

class SpiceSelectionPlugin(Plugin):

    def __init__(self, context):
        super(SpiceSelectionPlugin, self).__init__(context)

        self.setObjectName('SpiceSelectionPlugin')
        
        print("[SPICE SELECTION GUI:] Initialized")

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")  # Add argument(s) to the parser.
        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('spice_selection_gui'), 'resource', 'SpiceSelectionPlugin.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SpiceSelectionPluginUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)# Add widget to the user interface

        ## CUSTOM CODE STARTING HERE ##########################################################################

        # GUI  callbacks
        self._widget.pushButton_pepper.clicked[bool].connect(self._handle_pepper_clicked)
        self._widget.pushButton_salt.clicked[bool].connect(self._handle_salt_clicked)
        self._widget.pushButton_oil.clicked[bool].connect(self._handle_oil_clicked)
        self._widget.pushButton_vinegar.clicked[bool].connect(self._handle_vinegar_clicked)


        # Wait for service to be available
        #print("Waiting for service: idx_finder_server...")
        rospy.wait_for_service('idx_finder_server')

    def createRequest(self,target_bottle_name):

        # Create request
        request = LocalizeColorMaskRequest()
        request.color_profile = target_bottle_name

        # Create service handle
        color_mask_localization = rospy.ServiceProxy('idx_finder_server', LocalizeColorMask)

        # Call service
        response = color_mask_localization(request)
        rospy.loginfo("[SpiceUpSelectionGUI] : Received idx: "+str(response.idx))


    def _handle_pepper_clicked(self):
        rospy.loginfo("[SpiceUpSelectionGUI] : "+str("PEPPER selected"))
        self.createRequest("pepper")
        
    def _handle_salt_clicked(self):
        rospy.loginfo("[SpiceUpSelectionGUI] : "+str("SALT selected"))
        self.createRequest("salt")

    def _handle_oil_clicked(self):
        rospy.loginfo("[SpiceUpSelectionGUI] : "+str("OIL selected"))
        self.createRequest("oil")
    
    def _handle_vinegar_clicked(self):
        rospy.loginfo("[SpiceUpSelectionGUI] : "+str("VINEGAR selected"))
        self.createRequest("vinegar")
    
  