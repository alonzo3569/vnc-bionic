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
    TURTLE_POSE_TOPIC = '/turtle1/pose'

    # Constants that determine the behaviour of the dashboard
    # Pose is published at ~62 Hz; so we'll see ~30 sec of history. Note that
    # these parameters could be set through ROS parameters or services too!
    POSE_UPDATE_INTERVAL = 1
    POSE_MAX_TIMESTEPS = 2000
    POSE_ATTRIBUTES = ['x', 'y', 'theta', 'linear_velocity', 'angular_velocity']

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
        #self._pose_history_lock = Lock()
        self.plot_ch1 = np.zeros(self.fs * Dashboard.PLOT_TIME)

        # Setup the subscribers, action clients, etc.ntu_msgs/HydrophoneData
        self._voltage_sub = rospy.Subscriber('/hydrophone_data', HydrophoneData, self._voltage_cb)

        # Initialize the application
        self._define_app()

    @property
    def pose_history(self):
        return self._pose_history[:, :self._history_length]

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
        pose_graph_layout = html.Div(dcc.Graph(id='pose', style={ 'width': '100%' }, animate=True,), className='row')

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
            #print("HI")

            X = list(range(0,self.plot_ch1.size,1))
            Y = self.plot_ch1.tolist()
            print('X')
            print(len(X))
            print('Y')
            print(len(Y))
            X = signal.resample(X,len(X)/100)
            Y = signal.resample(Y,len(Y)/100)
            print('X after')
            print(len(X))
            print('Y after')
            print(len(Y))
            #X = list(range(0,60*1000,1)) # Upper bound of dash buffer : 100k
            #Y = list(range(0,60*1000,1))
            # plot
            data = go.Scatter(
                    x=X,
                    y=Y, # Y = [[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]]
                    name='Scatter',
                    mode= 'lines+markers'
                    )
        
            # return figure
            print(data)

            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    #fixedrange=True
                    range=[min(Y),max(Y)]
                ),
                xaxis=dict(
                    range=[min(X),max(X)]
                ),

                margin=dict(
                    autoexpand=True
                )
            )


            #return {'data': [data],'layout' : go.Layout(xaxis=dict(range=[min(X),max(X)]),
            #                                            yaxis=dict(range=[0,10]),)}

            return { 'data': [data], 'layout': layout }
        # Define callbacks to update the elements on the page
        #self._app.callback(
        #    dash.dependencies.Output('pose', 'figure'),
        #    [dash.dependencies.Input('interval-component', 'n_intervals')]
        #)(self._define_pose_history_callback())

#    def _define_pose_history_callback(self):
#        """
#        Define a callback that will be invoked on every update of the interval
#        component. Keep in mind that we return a callback here; not a result
#        """
#        def pose_history_callback(n_intervals):
#            # determine x value
#            tmp = [np.arange(1,11).tolist()]
#            X = [element+intervals for element in tmp[0]]
#            # Create the output graph
#            data = [
#                go.Scatter(
#                    name='Scatter',
#                    x=X,#self.plot_ch1,
#                    y=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],#list(range(0,self.plot_ch1.size,1)),
#                    mode='lines+markers'
#                )
#            ]
#            layout = go.Layout(
#                showlegend=True,
#                height=500,
#                yaxis=dict(
#                    fixedrange=True
#                ),
#                margin=dict(
#                    autoexpand=True
#                )
#            )
#            #print(list(range(0,self.plot_ch1.size,1)))
#
#            return { 'data': data, 'layout': layout }
#
#        return pose_history_callback

    def _voltage_cb(self, msg):
        """
        The callback for the position of the turtle on
        :const:`TURTLE_POSE_TOPIC`
        """
        tmp = np.array(msg.data_ch1)
        self.plot_ch1 = self.plot_ch1[tmp.size:]
        self.plot_ch1 = np.concatenate((self.plot_ch1,tmp),axis=0)
        #print(self.plot_ch1)
        #print('plot_ch1_size')
        #print(self.plot_ch1.size)
        #print('list size')
        #print(len(list(range(0,self.plot_ch1.size,1))))
        #print("msg recieved")

