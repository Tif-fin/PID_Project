import dash
import numpy as np 
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
from drone import Drone

class PIController:
    def __init__(self, Kp, Ki, setPoint) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.integral = 0
        self.setPoint = setPoint
    
    def update(self, measured_value, dt):
        error = self.setPoint - measured_value
        self.integral += error * dt
        return self.Kp * error + self.Ki * self.integral

    def auto_tune(self):
        # A simple auto-tune algorithm for demonstration purposes
        self.Kp = np.random.uniform(0.5, 2.5)
        self.Ki = np.random.uniform(0.1, 1.0)
        return self.Kp, self.Ki

# Initialize simulation parameters
dt = 0.01
total_time = 10
time = np.arange(0, total_time, dt)
drone = Drone(weight=10)
pi = PIController(Kp=2.0, Ki=0.5, setPoint=10)
altitudes = []
velocities = []
control_signals = []
errors = []

app = dash.Dash(__name__)

# Layout of the Dash app
app.layout = html.Div([
    html.H1("Drone Altitude Control using PI"),
    dcc.Graph(id='altitude-graph'),
    dcc.Graph(id='error-graph'),
    html.Div([
        html.Label('Kp:'),
        dcc.Slider(id='kp-slider', min=0, max=5, step=0.1, value=2),
        html.Label('Ki:'),
        dcc.Slider(id='ki-slider', min=0, max=2, step=0.1, value=0.5),
        html.Label('Setpoint:'),
        dcc.Input(id='setpoint-input', type='number', value=10),
        html.Button('Auto-Tune', id='auto-tune-button', n_clicks=0),
        html.Label('Weight:'),
        dcc.Slider(id='weight-slider', min=0.5, max=2.0, step=0.1, value=1.0)
    ]),
    dcc.Interval(id='interval-component', interval=100, n_intervals=0)  # Update every 100ms
])

# Callback to update PID parameters and run auto-tune
@app.callback(
    [Output('kp-slider', 'value'),
     Output('ki-slider', 'value')],
    [Input('auto-tune-button', 'n_clicks')],
    [State('kp-slider', 'value'),
     State('ki-slider', 'value')]
)
def auto_tune_parameters(n_clicks, current_kp, current_ki):
    if n_clicks > 0:
        new_kp, new_ki = pi.auto_tune()
        return new_kp, new_ki
    else:
        return current_kp, current_ki

# Callback to update the plot and PID parameters
@app.callback(
    Output('altitude-graph', 'figure'),
    [Input('kp-slider', 'value'),
     Input('ki-slider', 'value'),
     Input('setpoint-input', 'value'),
     Input('weight-slider', 'value'),
     Input('interval-component', 'n_intervals')]
)
def update_graph(Kp, Ki, setpoint, weight, n_intervals):
    global drone, pi, altitudes, velocities, control_signals

    # Update PID parameters
    pi.Kp = Kp
    pi.Ki = Ki
    pi.setPoint = setpoint
    drone.weight = weight

    # Run the simulation for one step
    control_signal = pi.update(drone.altitude, dt)
    drone.update(control_signal, dt)
    #error 
    errors.append(pi.setPoint-drone.altitude)

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
    
    fig.update_layout(title='Drone Altitude Adjustment Using PI Controller',
                      xaxis_title='Time (s)',
                      yaxis_title='Value',
                      yaxis=dict(range=[-15, 15]))
    
    return fig


# Callback to update the plot and PID parameters
@app.callback(
    Output('error-graph', 'figure'),
    [Input('interval-component', 'n_intervals')]
)
def update_graph( n_intervals):
    # Create the plot
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=list(time[:len(control_signals)]), y=control_signals, mode='lines', name='Control Signal'))
    fig.add_trace(go.Scatter(x=list(time[:len(errors)]), y=errors, mode='lines', name='Error')) 
    fig.update_layout(title='Error and Control Signal  of PI controller',
                      xaxis_title='Time (s)',
                      yaxis_title='Value',
                      yaxis=dict(range=[-20, 35]))
    
    return fig

if __name__ == "__main__":
    app.run_server(debug=True)
