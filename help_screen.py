#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from actionlib import SimpleActionClient
from monitored_navigation.ui_helper import UIHelper
from std_srvs.srv import Empty
from strands_webserver.msg import ModalDialogSrvAction, ModalDialogSrvGoal


class HelpScreen(UIHelper):

    def __init__(self):

        self.deployment_language = rospy.get_param("/deployment_language", "english")

        self.screen = SimpleActionClient('strands_webserver/modal_dialog', ModalDialogSrvAction)
        got_server = self.screen.wait_for_server(rospy.Duration(1))
        while not got_server:
            rospy.loginfo("help via screen is waiting for webserver modal dialog action...")
            got_server = self.screen.wait_for_server(rospy.Duration(1))
            if rospy.is_shutdown():
                return

        rospy.loginfo("help via screen got webserver modal dialog action")

        if self.deployment_language == "german":
            self.help_label = 'RETTE MICH'
            self.finish_label = 'OK'
            self.bumper_help_html = 'Hilfe! Ich bin in ein Hindernis gefahren. Wenn du mir weiterhelfen kannst, bitte drücke diesen Knopf.'
            self.nav_help_html = 'Hilfe! Ich habe Problem ein Hindernis zu passieren. Wenn du mir helfen kannst, bitte drücke diesen Knopf.'
            self.magnetic_stip_help_html = 'Ich bin versehentlich in eine unerlaubte Zone gefahren. Bitte benachrichtige meine Betreuer.'
            self.help_instructions_html = 'Dankeschön! Bitte folge diesen Anweisungen:'
            self.help_instructions_html += '<hr/>'
            self.help_instructions_html += '<ol>'
            self.help_instructions_html += '<li>Schiebe mich in eine freie Umgebung.</li>'
            self.help_instructions_html += '<li>Drücke den ' + self.finish_label + ' Knopf.</li>'
            self.help_instructions_html += '</ol>'
        else:
            self.help_label = 'HELP'
            self.finish_label = 'OK'
            self.bumper_help_html = 'Help! My bumper is stuck against something. If you can help me, please press the button below.'
            self.nav_help_html = 'Help! I appear to have trouble moving past an obstruction. If you can help me, please press the button below.'
            self.magnetic_stip_help_html = 'I feel very uncomfortable about moving in this area, please call one of my handlers.'
            self.help_instructions_html = 'Thank you! Please follow these instructions:'
            self.help_instructions_html += '<hr/>'
            self.help_instructions_html += '<ol>'
            self.help_instructions_html += '<li>Push me away from any obstructions into a clear space</li>'
            self.help_instructions_html += '<li>Press the ' + self.finish_label + ' button below</li>'
            self.help_instructions_html += '</ol>'

        UIHelper.__init__(self)

        rospy.loginfo("help via screen initialized")

    def ask_help(self, failed_component, interaction_service, n_fails):
        if failed_component == 'navigation':
            self.generate_help_content(self.help_label, self.nav_help_html,  interaction_service)
        elif failed_component == 'bumper':
            self.generate_help_content(self.help_label, self.bumper_help_html,  interaction_service)
        elif failed_component == 'magnetic_strip':
            self.generate_help_content(None, self.magnetic_stip_help_html,  interaction_service)

    def being_helped(self, failed_component, interaction_service, n_fails):
        self.generate_help_content(
            self.finish_label, self.help_instructions_html,  interaction_service)

    def help_finished(self, failed_component, interaction_service, n_fails):
        self.remove_help_content()

    def help_failed(self, failed_component, interaction_service, n_fails):
        self.remove_help_content()

    def remove_help_content(self):
        self.screen.cancel_all_goals()

    def generate_help_content(self, label, html, interaction_service):
        interaction_success = rospy.ServiceProxy(
            "/monitored_navigation/" + interaction_service, Empty)
        if label is None:
            goal = ModalDialogSrvGoal(title="Please help",
                                      text=html,
                                      buttons=[],
                                      buttons_class=[]
                                      )
        else:
            goal = ModalDialogSrvGoal(title="Please help",
                                      text=html,
                                      buttons=[label],
                                      buttons_class=['btn-success']
                                      )
        self.screen.send_goal(goal)
        self.screen.wait_for_result()
        result = self.screen.get_result()
        if result.button == label:
            interaction_success()
