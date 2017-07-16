#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from monitored_navigation.ui_helper import UIHelper
from std_srvs.srv import Empty


class HelpScreen(UIHelper):

    def __init__(self, socketio):

        self.socketio = socketio
        self.interaction_service = None
        self.help_button = 'HELP'
        self.finish_button = 'Done'
        self.bumper_help_text = 'Help! My bumper is stuck against something. Can you help me?'
        self.nav_help_text = 'Help! I appear to have trouble moving past an obstruction. Can you help me?'
        self.magnetic_stip_help_text = 'I feel very uncomfortable about moving in this area, please call one of my handlers.'
        self.help_instructions_text = 'Thank you! Please push me away from any obstructions into a clear space.'

        UIHelper.__init__(self)

        rospy.loginfo("[Waitress] Help via screen initialised")

    def ask_help(self, failed_component, interaction_service, n_fails):
        if failed_component == 'navigation':
            self.generate_help_content(self.help_button, self.nav_help_text,  interaction_service)
        elif failed_component == 'bumper':
            self.generate_help_content(
                self.help_button, self.bumper_help_text,  interaction_service)
        elif failed_component == 'magnetic_strip':
            self.generate_help_content(None, self.magnetic_stip_help_text,  interaction_service)

    def being_helped(self, failed_component, interaction_service, n_fails):
        self.generate_help_content(
            self.finish_button, self.help_instructions_text,  interaction_service)

    def help_finished(self, failed_component, interaction_service, n_fails):
        self.remove_help_content()

    def help_failed(self, failed_component, interaction_service, n_fails):
        self.remove_help_content()

    def remove_help_content(self):
        self.screen.cancel_all_goals()

    def generate_help_content(self, button, html, interaction_service):
        self.interaction_service = interaction_service
        if button is None:
            self.socketio.emit('helper',
                               dict(title="Please help",
                                    text=html),
                               namespace='/io')
        else:
            self.socketio.emit('helper',
                               dict(title="Please help",
                                    text=html,
                                    button=button),
                               namespace='/io')

    def helped_success(self):
        if self.interaction_service is not None:
            interaction_success = rospy.ServiceProxy(
                "/monitored_navigation/" + self.interaction_service, Empty)
            interaction_success()
        else:
            raise ValueError("No interaction service set")
