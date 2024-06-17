import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
import numpy as np 
from drone import Drone
class ProportionalController:
    def __init__(self, Kp, setPoint) -> None:
        '''Kp means Proportional Gain, Set Point'''
        self.kp = Kp
        self.setPoint = setPoint 

    def update(self, measured_value):
        error = self.setPoint - measured_value
        return error * self.kp

    def auto_tune(self):
        # A simple auto-tune algorithm for demonstration purposes
        self.kp = np.random.uniform(0.5, 2.5)
        return self.kp

# class Drone:
#     def __init__(self, altitude=0, velocity=0, weight=1.0):
#         self.altitude = altitude 
#         self.velocity = velocity
#         self.weight = weight

#     def update(self, thrust, dt):
#         gravity = 9.81 * self.weight
#         acceleration = (thrust - gravity) / self.weight
#         self.velocity += acceleration * dt
#         self.altitude += self.velocity * dt
#         if self.altitude < 0:
#             self.altitude = 0  # Ensure altitude is always positive

# Initialize the Dash app
app = dash.Dash(__name__)

# Initialize simulation parameters
dt = 0.01
total_time = 10
time = np.arange(0, total_time, dt)
drone = Drone(weight=1.0)
pc = ProportionalController(Kp=0.1, setPoint=10)
altitudes = []
velocities = []
control_signals = []
errors = []

# Layout of the Dash app
app.layout = html.Div([
    html.H1("Drone Altitude Control using Proportional Controller"),
    dcc.Graph(id='altitude-graph'),
    dcc.Graph(id='error-graph'),
    html.Div([
        html.Label('Kp:'),
        dcc.Slider(id='kp-slider', min=0, max=5, step=0.1, value=2),
        html.Label('Setpoint:'),
        dcc.Input(id='setpoint-input', type='number', value=10),
        html.Button('Auto-Tune', id='auto-tune-button', n_clicks=0),
        html.Label('Weight:'),
        dcc.Slider(id='weight-slider', min=0.5, max=2.0, step=0.1, value=1.0)
    ]),
    dcc.Interval(id='interval-component', interval=100, n_intervals=0)
])

# Callback to update controller parameters and run auto-tune
@app.callback(
    [Output('kp-slider', 'value')],
    [Input('auto-tune-button', 'n_clicks')],
    [State('kp-slider', 'value')]
)
def auto_tune_parameters(n_clicks, current_kp):
    if n_clicks > 0:
        new_kp = pc.auto_tune()
        return [new_kp]
    else:
        return [current_kp]

# Callback to update the plot and controller parameters
@app.callback(
    Output('altitude-graph', 'figure'),
    [Input('kp-slider', 'value'),
     Input('setpoint-input', 'value'),
     Input('weight-slider', 'value'),
     Input('interval-component', 'n_intervals')]
)
def update_graph(Kp, setpoint, weight, n_intervals):
    global drone, pc, altitudes, velocities, control_signals

    # Update controller parameters
    pc.kp = Kp
    pc.setPoint = setpoint
    drone.weight = weight

    # Run the simulation for one step
    control_signal = pc.update(drone.altitude)
    drone.update(control_signal, dt)
    #error 
    errors.append(pc.setPoint-drone.altitude)

    altitudes.append(drone.altitude)
    velocities.append(drone.velocity)
    control_signals.append(control_signal)
    
    if len(altitudes) > len(time):
        altitudes.pop(0)
        velocities.pop(0)
        control_signals.pop(0)

    # Create the plot
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=list(time[:len(altitudes)]), y=altitudes, mode='lines', name='Altitude'))
    fig.add_trace(go.Scatter(x=list(time[:len(velocities)]), y=velocities, mode='lines', name='Velocity'))
    fig.add_trace(go.Scatter(x=list(time[:len(control_signals)]), y=control_signals, mode='lines', name='Control Signal'))
    
    fig.update_layout(title='Drone Altitude Adjustment Using Proportional Controller',
                      xaxis_title='Time (s)',
                      yaxis_title='Value',
                      yaxis=dict(range=[-15, 15]))
    
    return fig

# Callback to update the plot and controller parameters
@app.callback(
    Output('error-graph', 'figure'),
     [Input('interval-component', 'n_intervals')]
)
def error_update_graph(n_intervals):
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=list(time[:len(control_signals)]), y=control_signals, mode='lines', name='Control Signal'))
    fig.add_trace(go.Scatter(x=list(time[:len(errors)]), y=errors, mode='lines', name='Error'))
    fig.update_layout(title='Error Plot',
                      xaxis_title='Time (s)',
                      yaxis_title='Value',
                      yaxis=dict(range=[-5, 20]))
    
    return fig



# Run the app
if __name__ == '__main__':
    app.run_server(debug=True)
