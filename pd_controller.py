import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
import numpy as np
from drone import Drone

# PID Controller Class
class PDController:
    def __init__(self, Kp, Kd, setpoint):
        self.Kp = Kp # proportional gain or constant
        self.Kd = Kd # derivative gain
        self.setpoint = setpoint # target point or set point
        self.previous_error = 0 # previous error

    def update(self, measured_value, dt):
        print(self.Kd,self.Kp)
        error = self.setpoint - measured_value # change in error
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error  + self.Kd * derivative
        self.previous_error = error
        return output

    def auto_tune(self):
        # A simple auto-tune algorithm for demonstration purposes
        self.Kp = np.random.uniform(0.5, 5.0)
        self.Kd = np.random.uniform(0.1, 10.0)
        return self.Kp, self.Kd

# Initialize the Dash app
app = dash.Dash(__name__)

# Layout of the Dash app
app.layout = html.Div([
    html.H1("Drone Altitude Control using PD"),
    dcc.Graph(id='altitude-graph'),
    dcc.Graph(id='error-graph'),  
    html.Div([
        html.Label('Kp:'),
        dcc.Slider(id='kp-slider', min=0, max=5, step=0.1, value=5),
        html.Label('Ki:'),
        dcc.Slider(id='kd-slider', min=0, max=5, step=0.1, value=6),
        html.Label('Setpoint:'),
        dcc.Input(id='setpoint-input', type='number', value=10),
        html.Button('Auto-Tune', id='auto-tune-button', n_clicks=0),
        html.Label('Weight:'),
        dcc.Slider(id='weight-slider', min=0.5, max=2.0, step=0.1, value=1.0)
    ]),
    dcc.Interval(id='interval-component', interval=100, n_intervals=0)  # Update every 100ms
])

# Initialize simulation parameters
dt = 0.01
total_time = 12
time = np.arange(0, total_time, dt)
drone = Drone(weight=9)
pid = PDController(Kp=2,  Kd=0.5, setpoint=10)
altitudes = []
velocities = []
control_signals = []
errors = []

# Callback to update PID parameters and run auto-tune
@app.callback(
    [Output('kp-slider', 'value'),
     Output('kd-slider', 'value')],
    [Input('auto-tune-button', 'n_clicks')],
    [State('kp-slider', 'value'),
     State('kd-slider', 'value')]
)
def auto_tune_parameters(n_clicks, current_kp, current_ki):
    if n_clicks > 0:
        new_kp, new_ki, new_kd = pid.auto_tune()
        return new_kp, new_ki, new_kd
    else:
        return current_kp, current_ki

# Callback to update the plot and PID parameters
@app.callback(
    Output('altitude-graph', 'figure'),
    [Input('kp-slider', 'value'),
     Input('kd-slider', 'value'),
     Input('setpoint-input', 'value'),
     Input('weight-slider', 'value'),
     Input('interval-component', 'n_intervals')]
)
def update_graph(Kp,  Kd, setpoint, weight, n_intervals):
    global drone, pid, altitudes, velocities, control_signals,errors

    # Update PID parameters
    pid.Kp = Kp
    pid.Kd = Kd
    pid.setpoint = setpoint
    drone.weight = weight

    # Run the simulation for one step
    control_signal = pid.update(drone.altitude, dt)
    drone.update(control_signal, dt)  
    # errors history 
    errors.append((pid.setpoint-drone.altitude))

    altitudes.append(drone.altitude)
    velocities.append(drone.velocity)
    control_signals.append(control_signal)
    
    if len(altitudes) > len(time):
        altitudes.pop(0)
        velocities.pop(0)
        control_signals.pop(0)
        errors.pop(0)
    
    # Create the plot
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=list(time[:len(altitudes)]), y=altitudes, mode='lines', name='Altitude'))
    fig.add_trace(go.Scatter(x=list(time[:len(velocities)]), y=velocities, mode='lines', name='Velocity'))
    fig.add_trace(go.Scatter(x=list(time[:len(control_signals)]), y=control_signals, mode='lines', name='Control Signal'))
    fig.update_layout(title='Drone Altitude Adjustment Using PD Controller',
                      xaxis_title='Time (s)',
                      yaxis_title='Value',
                      yaxis=dict(range=[-15, 20]))
    return fig

# Callback to update the error plot
@app.callback(
    Output('error-graph', 'figure'),
    [Input('interval-component', 'n_intervals')]
)
def update_error(n_intervals):
    # Create error graph
    error_fig = go.Figure()
    error_fig.add_trace(go.Scatter(x=list(time[:len(errors)]), y=errors, mode='lines', name='Error'))
    error_fig.add_trace(go.Scatter(x=list(time[:len(control_signals)]), y=control_signals, mode='lines', name='Control Signal'))
    error_fig.update_layout(title='Drone Altitude Adjustment Using PD Controller',
                      xaxis_title='Time (s)',
                      yaxis_title='Value',
                      yaxis=dict(range=[-15, 30]))
    return error_fig

# Run the app
if __name__ == '__main__':
    app.run_server(debug=True)
