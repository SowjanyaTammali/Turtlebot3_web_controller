const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const { exec } = require('child_process');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

app.use(express.static(__dirname));

// Robot state with enhanced position tracking
let robotState = {
    linearSpeed: 0.5,
    angularSpeed: 1.0,
    currentLinear: 0.0,
    currentAngular: 0.0,
    pose: {
        x: 0,
        y: 0,
        theta: 0
    },
    navigationStatus: 'Idle'
};

function getMapData() {
    exec('ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once', (error, stdout, stderr) => {
        if (error) {
            console.error('Error getting map data:', error);
            return;
        }
        try {
            console.log("Raw map data received");
            const lines = stdout.toString().split('\n');
            let width = 0, height = 0, resolution = 0.05;
            let data = [];
            let readingData = false;
            let origin = { x: -10, y: -10, z: 0 };  // Default origin

            for (const line of lines) {
                if (line.includes('width:')) {
                    width = parseInt(line.split(':')[1].trim());
                    console.log("Map width:", width);
                } else if (line.includes('height:')) {
                    height = parseInt(line.split(':')[1].trim());
                    console.log("Map height:", height);
                } else if (line.includes('resolution:')) {
                    resolution = parseFloat(line.split(':')[1].trim());
                    console.log("Map resolution:", resolution);
                } else if (line.includes('position:')) {
                    const matches = line.match(/x: ([-\d.]+).*y: ([-\d.]+).*z: ([-\d.]+)/);
                    if (matches) {
                        origin.x = parseFloat(matches[1]);
                        origin.y = parseFloat(matches[2]);
                        origin.z = parseFloat(matches[3]);
                        console.log("Map origin:", origin);
                    }
                }
            }

            console.log("Sending map data to clients:", { width, height, resolution, origin });
            broadcastToClients({
                type: 'map_data',
                data: {
                    width,
                    height,
                    resolution,
                    origin,
                    data: []  // We'll handle occupancy data later
                }
            });
        } catch (e) {
            console.error('Error parsing map data:', e);
        }
    });
}

function getLidarData() {
    exec('ros2 topic echo /scan sensor_msgs/msg/LaserScan --once', (error, stdout, stderr) => {
        if (error) {
            handleError(error, 'getting LIDAR data');
            return;
        }
        try {
            const scanData = parseScanData(stdout.toString());
            if (scanData) {
                broadcastToClients({
                    type: 'scan_data',
                    data: scanData
                });
            }
        } catch (e) {
            handleError(e, 'parsing LIDAR data');
        }
    });
}

function getRobotPose() {
    exec('ros2 topic echo /odom nav_msgs/msg/Odometry --once', (error, stdout, stderr) => {
        if (error) {
            handleError(error, 'getting robot pose');
            return;
        }
        try {
            const pose = parseOdomData(stdout.toString());
            robotState.pose = pose;
            broadcastToClients({
                type: 'robot_state',
                data: { pose }
            });
        } catch (e) {
            handleError(e, 'parsing robot pose');
        }
    });
}

function checkNavigationStatus() {
    exec('ros2 action list -t', (error, stdout, stderr) => {
        if (error) {
            console.error('Error checking navigation status:', error);
            return;
        }
        try {
            const actions = stdout.toString().split('\n');
            const isNavigating = actions.some(action => 
                action.includes('/navigate_to_pose/_action/status') ||
                action.includes('/NavigateToPose/_action/status')
            );
            
            const status = isNavigating ? 'Moving to goal' : 'Idle';
            broadcastToClients({
                type: 'nav_status',
                status: status
            });
        } catch (e) {
            console.error('Error processing navigation status:', e);
        }
    });
}

function parseScanData(data) {
    const lines = data.split('\n');
    let ranges = [];
    let angle_min = 0, angle_max = 0, angle_increment = 0;
    let range_min = 0, range_max = 0;

    for (const line of lines) {
        if (line.includes('angle_min:')) {
            angle_min = parseFloat(line.split(':')[1].trim());
        } else if (line.includes('angle_max:')) {
            angle_max = parseFloat(line.split(':')[1].trim());
        } else if (line.includes('angle_increment:')) {
            angle_increment = parseFloat(line.split(':')[1].trim());
        } else if (line.includes('range_min:')) {
            range_min = parseFloat(line.split(':')[1].trim());
        } else if (line.includes('range_max:')) {
            range_max = parseFloat(line.split(':')[1].trim());
        } else if (line.includes('ranges:')) {
            const nextLine = lines[lines.indexOf(line) + 1];
            if (nextLine) {
                ranges = nextLine.split(',').map(Number);
            }
        }
    }

    return {
        ranges,
        angle_min,
        angle_max,
        angle_increment,
        range_min,
        range_max
    };
}

function parseOdomData(data) {
    const lines = data.split('\n');
    let x = 0, y = 0, theta = 0;

    for (const line of lines) {
        if (line.includes('position:')) {
            const posMatch = line.match(/x: ([-\d.]+).*y: ([-\d.]+)/);
            if (posMatch) {
                x = parseFloat(posMatch[1]);
                y = parseFloat(posMatch[2]);
            }
        } else if (line.includes('orientation:')) {
            const orientMatch = line.match(/z: ([-\d.]+).*w: ([-\d.]+)/);
            if (orientMatch) {
                const z = parseFloat(orientMatch[1]);
                const w = parseFloat(orientMatch[2]);
                theta = 2 * Math.atan2(z, w);
            }
        }
    }

    return { x, y, theta };
}

function publishVelocity(linear, angular) {
    const cmd = `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{\
        linear: {x: ${linear}, y: 0.0, z: 0.0},\
        angular: {x: 0.0, y: 0.0, z: ${angular}}\
    }"`;
    
    robotState.currentLinear = linear;
    robotState.currentAngular = angular;

    exec(cmd, (error, stdout, stderr) => {
        if (error) {
            handleError(error, 'publishing velocity');
            return;
        }
        broadcastToClients({
            type: 'movement',
            velocities: { linear, angular }
        });
    });
}

function updateSpeeds(command) {
    const speedChangeRate = 1.1;
    switch(command) {
        case 'q':
            robotState.linearSpeed *= speedChangeRate;
            robotState.angularSpeed *= speedChangeRate;
            break;
        case 'z':
            robotState.linearSpeed /= speedChangeRate;
            robotState.angularSpeed /= speedChangeRate;
            break;
        case 'w':
            robotState.linearSpeed *= speedChangeRate;
            break;
        case 'x':
            robotState.linearSpeed /= speedChangeRate;
            break;
        case 'e':
            robotState.angularSpeed *= speedChangeRate;
            break;
        case 'c':
            robotState.angularSpeed /= speedChangeRate;
            break;
    }

    // Enforce speed limits
    robotState.linearSpeed = Math.min(Math.max(robotState.linearSpeed, 0.1), 2.0);
    robotState.angularSpeed = Math.min(Math.max(robotState.angularSpeed, 0.1), 2.0);

    broadcastToClients({
        type: 'speedUpdate',
        speeds: robotState
    });
}

function handleMovement(key) {
    let linear = 0;
    let angular = 0;
    
    switch(key) {
        case 'u':
            linear = robotState.linearSpeed;
            angular = robotState.angularSpeed;
            break;
        case 'i':
            linear = robotState.linearSpeed;
            break;
        case 'o':
            linear = robotState.linearSpeed;
            angular = -robotState.angularSpeed;
            break;
        case 'j':
            angular = robotState.angularSpeed;
            break;
        case 'l':
            angular = -robotState.angularSpeed;
            break;
        case 'm':
            linear = -robotState.linearSpeed;
            angular = robotState.angularSpeed;
            break;
        case ',':
            linear = -robotState.linearSpeed;
            break;
        case '.':
            linear = -robotState.linearSpeed;
            angular = -robotState.angularSpeed;
            break;
        case 'k':
            linear = 0;
            angular = 0;
            break;
    }

    publishVelocity(linear, angular);
    return { linear, angular };
}

function broadcastToClients(message) {
    wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify(message));
        }
    });
}

function handleError(error, operation) {
    console.error(`Error during ${operation}:`, error);
    broadcastToClients({
        type: 'error',
        message: `Failed to ${operation}: ${error.message}`
    });
}

// Set up periodic updates
setInterval(() => {
    getMapData();
    getLidarData();
    getRobotPose();
    checkNavigationStatus();
}, 1000);

wss.on('connection', (ws) => {
    console.log('Client connected!');
    
    ws.send(JSON.stringify({ 
        type: 'status', 
        speeds: robotState,
        message: 'Connected to ROS2 server'
    }));

    ws.on('message', (message) => {
        try {
            console.log('Received message:', message.toString());
            const data = JSON.parse(message);
            
            switch (data.type) {
                case 'keyCommand':
                    if (['q', 'z', 'w', 'x', 'e', 'c'].includes(data.key)) {
                        updateSpeeds(data.key);
                    } else {
                        const result = handleMovement(data.key);
                        ws.send(JSON.stringify({
                            type: 'movement',
                            velocities: result
                        }));
                    }
                    break;
                
                case 'set_initial_pose':
                    console.log('Setting initial pose:', data.x, data.y, data.theta);
                    const initCmd = `ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
                        header: {
                            frame_id: 'map',
                            stamp: {
                                sec: 0,
                                nanosec: 0
                            }
                        },
                        pose: {
                            pose: {
                                position: {
                                    x: ${data.x},
                                    y: ${data.y},
                                    z: 0.0
                                },
                                orientation: {
                                    x: 0.0,
                                    y: 0.0,
                                    z: ${Math.sin(data.theta/2)},
                                    w: ${Math.cos(data.theta/2)}
                                }
                            },
                            covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
                        }
                    }"`;
                    
                    exec(initCmd, (error, stdout, stderr) => {
                        if (error) {
                            console.error('Error setting initial pose:', error);
                            return;
                        }
                        console.log('Initial pose set:', stdout);
                    });
                    break;
                                    
                case 'set_nav_goal':
                    console.log('Setting navigation goal:', data.x, data.y, data.theta);
    
                    // Use ROS2 action server for navigation
                    const navCmd = `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
                        pose: {
                            header: {
                                frame_id: 'map',
                                stamp: {
                                    sec: 0,
                                    nanosec: 0
                                }
                            },
                            pose: {
                                position: {
                                    x: ${data.x},
                                    y: ${data.y},
                                    z: 0.0
                                },
                                orientation: {
                                    x: 0.0,
                                    y: 0.0,
                                    z: ${Math.sin(data.theta/2)},
                                    w: ${Math.cos(data.theta/2)}
                                }
                            }
                        }
                    }" --feedback`;
                    
                    exec(navCmd, (error, stdout, stderr) => {
                        if (error) {
                            console.error('Error setting navigation goal:', error);
                            return;
                        }
                        console.log('Navigation goal set:', stdout);
                        broadcastToClients({
                            type: 'nav_status',
                            status: 'Moving to goal'
                        });
                    });
                    break;

                case 'cancel_navigation':
                    exec('ros2 action send_goal --cancel /navigate_to_pose', (error, stdout, stderr) => {
                        if (error) {
                            handleError(error, 'canceling navigation');
                            return;
                        }
                        console.log('Navigation canceled');
                        robotState.navigationStatus = 'Idle';
                        broadcastToClients({
                            type: 'nav_status',
                            status: 'Idle'
                        });
                    });
                    break;
            }
        } catch (error) {
            handleError(error, 'processing message');
        }
    });

    ws.on('close', () => {
        console.log('Client disconnected!');
        publishVelocity(0, 0);  // Stop robot when client disconnects
    });

    ws.on('error', (error) => {
        console.error('WebSocket error:', error);
    });
});

// Function to check ROS2 system status
function checkROS2Status() {
    return new Promise((resolve, reject) => {
        exec('ros2 topic list', (error, stdout, stderr) => {
            if (error) {
                console.error('ROS2 system not available:', error);
                resolve(false);
            } else {
                console.log('Available ROS2 topics:', stdout);
                resolve(true);
            }
        });
    });
}

// Startup checks
async function performStartupChecks() {
    const ros2Available = await checkROS2Status();
    if (!ros2Available) {
        console.error('ROS2 system is not available. Please make sure:');
        console.error('1. You have sourced ROS2: source /opt/ros/humble/setup.bash');
        console.error('2. Gazebo is running with TurtleBot3');
        console.error('3. Nav2 is running');
        process.exit(1);
    }
    
    console.log('ROS2 system is available. Starting server...');
}

// Initialize the server
performStartupChecks();

const PORT = process.env.PORT || 8080;
server.listen(PORT, '0.0.0.0', () => {
    console.log(`Server running at http://0.0.0.0:${PORT}`);
    console.log('To access from other devices, use your computer\'s IP address');
});

// Handle process termination
process.on('SIGINT', () => {
    console.log('Shutting down server...');
    publishVelocity(0, 0);  // Stop robot before shutting down
    wss.clients.forEach(client => {
        client.close();
    });
    process.exit(0);
});