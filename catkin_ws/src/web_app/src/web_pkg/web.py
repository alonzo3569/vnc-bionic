#!/usr/bin/env python
# Create a dash server that is also an actionlib client to turtle_actionlib

from __future__ import print_function, division

import os
import sys
import time

import numpy as np


import rospy
import rospkg


# Plotly, Dash, and Flask
import plotly.graph_objs as go

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output

from flask import jsonify

from ntu_msgs.msg import HydrophoneData, HydrophoneFFTData
from scipy import signal

# The app definition

APP = dash.Dash(
    __name__
    #external_stylesheets=[
    #    {
    #        'href': 'https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/css/bootstrap.min.css',
    #        'rel': 'stylesheet',
    #        'integrity': 'sha384-ggOyR0iXCbMQv3Xipma34MD+dH/1fQ784/j6cY/iJTQUOhcWr7x9JvoRxT2MZw1T',
    #        'crossorigin': 'anonymous',
    #    },
    #]
)


class Dashboard(object):
    """
    Create a Flask server to display the UI and a ROS node to send commands to
    the turtlesim
    """

    # Flask
    APP_HOST = '0.0.0.0'
    APP_PORT = 8080

    # Actions, Topics, and Services
    # Note that although the values are hard-coded for now, these can be set via
    # service or ROS params if need be (a trivial update)

    # Constants that determine the behaviour of the dashboard
    # Pose is published at ~62 Hz; so we'll see ~30 sec of history. Note that
    # these parameters could be set through ROS parameters or services too!
    POSE_UPDATE_INTERVAL = 0.5
    POSE_MAX_TIMESTEPS = 2000

    # Plot Constants
    SAMPLE_RATE_TOPIC = '/get_sound_data/pcm_sampleRate' 
    PLOT_TIME = 5 # (sec)

    def __init__(self):
        global APP

        # The Flask application
        self._app = APP

        # OnstartUp
        self.fs = rospy.get_param(Dashboard.SAMPLE_RATE_TOPIC)

        # Initialize the variables that we'll be using to save information
        self.plot_ch1 = np.zeros(self.fs * Dashboard.PLOT_TIME)

        self.time_axis = list(range(0,self.plot_ch1.size,1))
        self.time_axis = [element/self.fs for element in self.time_axis]
        self.division = 50 # downsample 
        self.time_axis_downsample = self.time_axis[::self.division]

        # Setup the subscribers, action clients, etc.ntu_msgs/HydrophoneData
        self._voltage_sub = rospy.Subscriber('/hydrophone_data', HydrophoneData, self._voltage_cb)

        # Initialize the application
        self._define_app()


    def start(self):
        self._app.run_server(host=Dashboard.APP_HOST,
                             port=Dashboard.APP_PORT,
                             debug=False)
        rospy.loginfo("...turtle_shape connected.")

    def stop(self, *args, **kwargs):
        # Give some time for rospy to shutdown (cannot use rospy now!)
        print("Shutting down Dash server")
        time.sleep(2)
        sys.exit(0)

    def _define_app(self):
        """
        Define the app layout and callbacks here
        """
        # Define each component of the page

        # First the graph element that will plot the pose and velocity of the
        # robot
        pose_graph_layout = html.Div(dcc.Graph(id='pose', style={ 'width': '100%' },), className='row')

        # String them all together in a single page
        self._app.layout = html.Div(
            [
                # The section showing the action status
                html.Div(html.H3('Real-Time Volts:', className='col'), className='row my-2'),
                pose_graph_layout,

                # The interval component to update the plots
                dcc.Interval(id='interval-component',
                             n_intervals=0,
                             interval=(Dashboard.POSE_UPDATE_INTERVAL * 1000)),
            ],
            className="container"
        )


        # Connect the plotly graphs with Dash components
        @APP.callback(
            Output('pose','figure'), # Don't use [] if there's only one output
            [Input('interval-component','n_intervals')]
        )
        def plot_volts_cb(n_intervals):

            Y = self.plot_ch1.tolist()
            Y_down = signal.resample(Y,int(len(Y)/self.division))
            #print('time_axis')
            #print(len(self.time_axis))
            #print('time_axis downsample')
            #print(len(self.time_axis_downsample))

            #print('Y_axis')
            #print(len(Y))
            #print('Y_axis downsample')
            #print(len(Y_down))
            #print(type(Y_down))
            # Upper bound of dash buffer : 100k


            # plot
            data = go.Scatter(
                    x=self.time_axis_downsample,
                    y=Y_down.tolist(), 
                    name='Scatter',
                    mode= 'lines'
                    )
        
            print(data)

            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    #fixedrange=True
                    range=[-0.6,0.6]
                ),
                xaxis=dict(
                    range=[min(self.time_axis_downsample),max(self.time_axis_downsample)]
                ),

                margin=dict(
                    autoexpand=True
                )
            )


            return { 'data': [data], 'layout': layout }

    def _voltage_cb(self, msg):

        # Get subscribe msg
        tmp = np.array(msg.data_ch1)

        # Remove previous data & append msg to array
        self.plot_ch1 = self.plot_ch1[tmp.size:]
        self.plot_ch1 = np.concatenate((self.plot_ch1,tmp),axis=0)

