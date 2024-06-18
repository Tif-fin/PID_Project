import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
import numpy as np

# PID Controller Class
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

    def auto_tune(self, drone, dt, num_iterations=100):
        best_kp, best_ki, best_kd = self.Kp, self.Ki, self.Kd
        best_performance = float('inf')
        
        for _ in range(num_iterations):
            kp = np.random.uniform(0.5, 2.5)
            ki = np.random.uniform(0.1, 1.0)
            kd = np.random.uniform(0.1, 2.0)
            self.Kp, self.Ki, self.Kd = kp, ki, kd

            performance = self.evaluate_performance(drone, dt)
            if performance < best_performance:
                best_performance = performance
                best_kp, best_ki, best_kd = kp, ki, kd

        self.Kp, self.Ki, self.Kd = best_kp, best_ki, best_kd
        return best_kp, best_ki, best_kd

    def evaluate_performance(self, drone, dt, num_steps=100):
        total_error = 0
        test_drone = Drone(altitude=drone.altitude, velocity=drone.velocity, weight=drone.weight)
        for _ in range(num_steps):
            control_signal = self.update(test_drone.altitude, dt)
            test_drone.update(control_signal, dt)
            total_error += abs(self.setpoint - test_drone.altitude)
        return total_error

# Simulate Drone Dynamics
class Drone:
    def __init__(self, altitude=0, velocity=0, weight=1.0):
        self.altitude = altitude
        self.velocity = velocity
        self.weight = weight

    def update(self, thrust, dt):
        gravity = 9.81 * self.weight
        acceleration = (thrust - gravity) / self.weight
        self.velocity += acceleration * dt
        self.altitude += self.velocity * dt

# Initialize the Dash app
app = dash.Dash(__name__)

# Layout of the Dash app
app.layout = html.Div([
    html.H1("Drone Altitude Control using PID"),
    dcc.Graph(id='altitude-graph'),
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
    html.Button('Start Simulation', id='start-button', n_clicks=0),
    html.Button('Stop Simulation', id='stop-button', n_clicks=0),
    dcc.Interval(id='interval-component', interval=100, n_intervals=0)  # Update every 100ms
])

# Initialize simulation parameters
dt = 0.01
total_time = 10
time = np.arange(0, total_time, dt)
drone = Drone()
pid = PIDController(Kp=2.0, Ki=0.5, Kd=1.0, setpoint=10)
altitudes = []
velocities = []
control_signals = []
simulation_running = False

# Callback to update PID parameters and run auto-tune
@app.callback(
    [Output('kp-slider', 'value'),
     Output('ki-slider', 'value'),
     Output('kd-slider', 'value')],
    [Input('auto-tune-button', 'n_clicks')],
    [State('kp-slider', 'value'),
     State('ki-slider', 'value'),
     State('kd-slider', 'value'),
     State('setpoint-input', 'value'),
     State('weight-slider', 'value')]
)
def auto_tune_parameters(n_clicks, current_kp, current_ki, current_kd, setpoint, weight):
    if n_clicks > 0:
        drone.weight = weight
        pid.setpoint = setpoint
        new_kp, new_ki, new_kd = pid.auto_tune(drone, dt)
        return new_kp, new_ki, new_kd
    else:
        return current_kp, current_ki, current_kd

# Callback to start and stop the simulation
@app.callback(
    Output('interval-component', 'disabled'),
    [Input('start-button', 'n_clicks'),
     Input('stop-button', 'n_clicks')]
)
def toggle_simulation(start_clicks, stop_clicks):
    global simulation_running
    if start_clicks > stop_clicks:
        simulation_running = True
        return False  # Enable interval updates
    else:
        simulation_running = False
        return True  # Disable interval updates

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
    global drone, pid, altitudes, velocities, control_signals, simulation_running

    if not simulation_running:
        return go.Figure()  # Return an empty figure if simulation is not running

    # Update PID parameters
    pid.Kp = Kp
    pid.Ki = Ki
    pid.Kd = Kd
    pid.setpoint = setpoint
    drone.weight = weight

    # Run the simulation for one step
    control_signal = pid.update(drone.altitude, dt)
    drone.update(control_signal, dt)
    
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
    
    fig.update_layout(title='Drone Altitude Adjustment Using PID Controller',
                      xaxis_title='Time (s)',
                      yaxis_title='Value',
                      yaxis=dict(range=[-15, 15]))
    
    return fig

# Run the app
if __name__ == '__main__':
    app.run_server(debug=True)
