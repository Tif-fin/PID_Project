import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
import numpy as np
from drone import Drone

# PID Controller Class
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp # proportional gain or constant
        self.Ki = Ki # Integral gain
        self.Kd = Kd # derivative gain
        self.setpoint = setpoint # target point or set point
        self.integral = 0 # integral value at the first 
        self.previous_error = 0 # previous error

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value # change in error
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return (output,self.Kp * error,self.Ki * self.integral,self.Kd * derivative)

    def auto_tune(self):
        # A simple auto-tune algorithm for demonstration purposes
        self.Kp = np.random.uniform(0.5, 1.0)
        self.Ki = np.random.uniform(0.5, 2.0)
        self.Kd = np.random.uniform(0.1, 2.0)
        return self.Kp, self.Ki, self.Kd

# Initialize the Dash app
app = dash.Dash(__name__)

# Layout of the Dash app
app.layout = html.Div([
    html.H1("Drone Altitude Control using PID"),
    dcc.Graph(id='altitude-graph'),
    dcc.Graph(id='error-graph'), 
    html.Div([
        html.Label('Kp:'),
        dcc.Slider(id='kp-slider', min=0, max=5, step=0.1, value=2),
        html.Label('Ki:'),
        dcc.Slider(id='ki-slider', min=0, max=2, step=0.1, value=0.5),
        html.Label('Kd:'),
        dcc.Slider(id='kd-slider', min=0, max=5, step=0.1, value=1),
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
drone = Drone()
pid = PIDController(Kp=0.6, Ki=0.2, Kd=0.1, setpoint=10)
altitudes = []
velocities = []
control_signals = []
errors = []
kps =[]
kis = []
kds =[]
# Callback to update PID parameters and run auto-tune
@app.callback(
    [Output('kp-slider', 'value'),
     Output('ki-slider', 'value'),
     Output('kd-slider', 'value')],
    [Input('auto-tune-button', 'n_clicks')],
    [State('kp-slider', 'value'),
     State('ki-slider', 'value'),
     State('kd-slider', 'value')]
)
def auto_tune_parameters(n_clicks, current_kp, current_ki, current_kd):
    if n_clicks > 0:
        new_kp, new_ki, new_kd = pid.auto_tune()
        return new_kp, new_ki, new_kd
    else:
        return current_kp, current_ki, current_kd

# Callback to update the plot and PID parameters
@app.callback(
    Output('altitude-graph', 'figure'),
    [Input('kp-slider', 'value'),
     Input('ki-slider', 'value'),
     Input('kd-slider', 'value'),
     Input('setpoint-input', 'value'),
     Input('weight-slider', 'value'),
     Input('interval-component', 'n_intervals')]
)
def update_graph(Kp, Ki, Kd, setpoint, weight, n_intervals):
    global drone, pid, altitudes, velocities, control_signals,errors

    # Update PID parameters
    pid.Kp = Kp
    pid.Ki = Ki
    pid.Kd = Kd
    pid.setpoint = setpoint
    drone.weight = weight


    # Calculate elapsed time
    elapsed_time = n_intervals * dt

    # Stop updating if elapsed time exceeds total_time
    if elapsed_time > total_time:
        return dash.no_update, True

    # Run the simulation for one step
    control_signal,kp,ki,kd = pid.update(drone.altitude, dt)
    #append 
    kps.append(kp)
    kis.append(ki)
    kds.append(kd)

    drone.update(control_signal, dt)
    #error 
    errors.append(pid.previous_error)
    altitudes.append(drone.altitude)
    velocities.append(drone.velocity)
    control_signals.append(control_signal)
    
    if len(altitudes) > len(time):
        altitudes.pop(0)
        velocities.pop(0)
        control_signals.pop(0)
    
    # Create the plot
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=list(time[:len(kps)]), y=kps, mode='lines', name='KP'))
    fig.add_trace(go.Scatter(x=list(time[:len(kis)]), y=kis, mode='lines', name='KI'))
    fig.add_trace(go.Scatter(x=list(time[:len(kds)]), y=kds, mode='lines', name='KD'))
    fig.add_trace(go.Scatter(x=list(time[:len(errors)]), y=errors, mode='lines', name='Error'))
    fig.update_layout(title='Drone Altitude Adjustment Using PID Controller',
                      xaxis_title='Time (s)',
                      yaxis_title='Value',
                      yaxis=dict(range=[-15, 20]))
    
    return fig





# Run the app
if __name__ == '__main__':
    app.run_server(debug=True)
