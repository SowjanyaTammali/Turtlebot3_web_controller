const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const { exec } = require('child_process');

process.on('uncaughtException', (err) => {
    console.error('Uncaught Exception:', err);
});

process.on('unhandledRejection', (reason, promise) => {
    console.error('Unhandled Rejection at:', promise, 'reason:', reason);
});

// Define known positions 
const ROBOT_START_POSITION = {
    x: -2.0,    // Default starting position 
    y: -0.5,
    theta: 0.0
};

const TARGET_POSITION = {
    x: 2.0,     // A clear location in the map 
    y: 2.0,     // Adjust these values based on your map 
    theta: 0.0
};

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

let navigationStartTime = null;
let lastStatus = null;

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
            let origin = { x: -10, y: -10, z: 0 };

            for (const line of lines) {
                if (line.includes('width:')) {
                    width = parseInt(line.split(':')[1].trim());
                } else if (line.includes('height:')) {
                    height = parseInt(line.split(':')[1].trim());
                } else if (line.includes('resolution:')) {
                    resolution = parseFloat(line.split(':')[1].trim());
                } else if (line.includes('position:')) {
                    const matches = line.match(/x: ([-\d.]+).*y: ([-\d.]+).*z: ([-\d.]+)/);
                    if (matches) {
                        origin.x = parseFloat(matches[1]);
                        origin.y = parseFloat(matches[2]);
                        origin.z = parseFloat(matches[3]);
                    }
                } else if (line.includes('data:')) {
                    readingData = true;
                } else if (readingData && line.trim()) {
                    const value = parseInt(line.trim());
                    if (!isNaN(value)) {
                        data.push(value);
                    }
                }
            }

            broadcastToClients({
                type: 'map_data',
                data: {
                    width,
                    height,
                    resolution,
                    origin,
                    data: data
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

// Improved navigation status monitoring that matches RViz
function monitorNavigationStatus() {
    exec('ros2 topic echo /navigate_to_pose/_action/feedback --once', (error, stdout) => {
        let status = {
            navigation: 'undefined',
            localization: 'undefined',
            feedback: 'undefined',
            eta: 'calculating...',
            distance: 'calculating...',
            timeTaken: '0 s',
            recoveries: '0'
        };

        if (!error && stdout) {
            console.log('Raw feedback:', stdout);
            const feedbackData = stdout.toString();

            // Parse the exact values using regex for precision
            const distanceMatch = feedbackData.match(/distance_remaining: ([\d.]+)/);
            const navigationTimeMatch = feedbackData.match(/navigation_time:\s+sec: (\d+)/);
            const etaMatch = feedbackData.match(/estimated_time_remaining:\s+sec: (\d+)/);
            const recoveriesMatch = feedbackData.match(/number_of_recoveries: (\d+)/);

            if (distanceMatch) {
                const distance = parseFloat(distanceMatch[1]);
                console.log('Exact distance:', distance);
                status.distance = distance.toFixed(2) + ' m';
                
                if (distance < 0.1) {
                    status.navigation = 'reached';
                    status.feedback = 'reached';
                    status.eta = '0 s';
                } else {
                    status.navigation = 'active';
                    status.feedback = 'active';
                }
            }

            if (navigationTimeMatch) {
                const navTime = parseInt(navigationTimeMatch[1]);
                console.log('Exact navigation time:', navTime);
                status.timeTaken = navTime + ' s';
            }

            if (etaMatch) {
                const eta = parseInt(etaMatch[1]);
                console.log('Exact ETA:', eta);
                status.eta = eta + ' s';
            }

            if (recoveriesMatch) {
                status.recoveries = recoveriesMatch[1];
                console.log('Exact recoveries:', status.recoveries);
            }

            // Check localization (AMCL)
            exec('ros2 topic echo /amcl_pose --once', (error2, stdout2) => {
                status.localization = (!error2 && stdout2) ? 'active' : 'undefined';
                
                console.log('Sending exact status:', status);
                broadcastToClients({
                    type: 'nav_status',
                    data: status
                });
            });
        } else {
            // If no feedback, check if we're done
            exec('ros2 action status navigate_to_pose --spin-time 0', (statusError, statusOutput) => {
                if (!statusError && statusOutput && statusOutput.includes('succeeded')) {
                    status.navigation = 'reached';
                    status.feedback = 'reached';
                    status.eta = '0 s';
                    status.distance = '0.00 m';
                }

                exec('ros2 topic echo /amcl_pose --once', (error2, stdout2) => {
                    status.localization = (!error2 && stdout2) ? 'active' : 'undefined';
                    broadcastToClients({
                        type: 'nav_status',
                        data: status
                    });
                });
            });
        }
    });
}

function debugNavigation() {
    console.log('Checking navigation topics...');
    
    // List all navigation-related topics
    exec('ros2 topic list | grep -E "navigate|feedback|status"', (error, stdout) => {
        if (!error && stdout) {
            console.log('Available navigation topics:', stdout);
            
            // Echo each topic briefly
            stdout.split('\n').forEach(topic => {
                if (topic) {
                    exec(`ros2 topic echo ${topic} --once`, (err, out) => {
                        if (!err && out) {
                            console.log(`Data from ${topic}:`, out);
                        }
                    });
                }
            });
        }
    });
}



// Improved initial pose setting
// Replace the existing setInitialPose function with this improved version
async function setInitialPose() {
    // First, clear any existing navigation goals
    await new Promise((resolve) => {
        exec('ros2 action send_goal --cancel /navigate_to_pose', () => {
            resolve();
        });
    });

    // Wait a moment for the cancellation to take effect
    await new Promise(resolve => setTimeout(resolve, 500));

    // Create the initialpose command with explicit quaternion calculation
    const yaw = ROBOT_START_POSITION.theta;
    const quaternionZ = Math.sin(yaw / 2);
    const quaternionW = Math.cos(yaw / 2);

    const cmd = `ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
        header: {
            stamp: {
                sec: 0,
                nanosec: 0
            },
            frame_id: 'map'
        },
        pose: {
            pose: {
                position: {
                    x: ${ROBOT_START_POSITION.x},
                    y: ${ROBOT_START_POSITION.y},
                    z: 0.0
                },
                orientation: {
                    x: 0.0,
                    y: 0.0,
                    z: ${quaternionZ},
                    w: ${quaternionW}
                }
            },
            covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        }
    }" --qos-reliability reliable --qos-durability transient_local --once`;

    return new Promise((resolve, reject) => {
        exec(cmd, (error) => {
            if (error) {
                console.error('Error setting initial pose:', error);
                reject(error);
                return;
            }

            // After setting pose, verify it was received
            verifyPoseSet(5); // Try up to 5 times
            resolve();
        });
    });
}

// Add this verification function
function verifyPoseSet(maxAttempts = 5) {
    let attempts = 0;
    const checkInterval = setInterval(() => {
        exec('ros2 topic echo /amcl_pose geometry_msgs/msg/PoseWithCovarianceStamped --once', (error, stdout) => {
            attempts++;
            if (!error && stdout) {
                // Successfully got AMCL pose
                clearInterval(checkInterval);
                broadcastToClients({
                    type: 'nav_status',
                    data: {
                        navigation: 'undefined',
                        localization: 'active',
                        feedback: 'undefined',
                        eta: 'N/A',
                        distance: 'N/A',
                        timeTaken: '0 s',
                        recoveries: '0'
                    }
                });
            } else if (attempts >= maxAttempts) {
                // Failed to verify pose after max attempts
                clearInterval(checkInterval);
                console.error('Failed to verify pose set');
            }
        });
    }, 1000);
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
    const cmd = `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{
        linear: {x: ${linear}, y: 0.0, z: 0.0},
        angular: {x: 0.0, y: 0.0, z: ${angular}}
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
    switch (command) {
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

    switch (key) {
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

function checkROS2Navigation() {
    console.log('Checking ROS2 navigation system...');
    
    // Check available actions
    exec('ros2 action list', (error, stdout) => {
        console.log('Available actions:', stdout);
        
        // Check if navigate_to_pose action exists
        if (stdout.includes('navigate_to_pose')) {
            console.log('navigate_to_pose action found');
            
            // Check action status
            exec('ros2 action status navigate_to_pose', (error2, stdout2) => {
                console.log('Navigation action status:', stdout2);
            });
        } else {
            console.error('navigate_to_pose action not found!');
        }
    });
}

function checkTopics() {
    console.log('Checking available topics...');
    exec('ros2 topic list', (error, stdout) => {
        if (error) {
            console.error('Error getting topics:', error);
            return;
        }
        console.log('Available topics:', stdout);
        
        // Check specific navigation topics
        const topicsToCheck = [
            '/navigate_to_pose/_action/feedback',
            '/navigate_to_pose/_action/status',
            '/amcl_pose'
        ];
        
        topicsToCheck.forEach(topic => {
            exec(`ros2 topic info ${topic}`, (error, info) => {
                if (!error) {
                    console.log(`Topic ${topic} info:`, info);
                }
            });
        });
    });
}


// Set up periodic updates 
setInterval(() => {
    getMapData();
    getLidarData();
    getRobotPose();
   monitorNavigationStatus();
    debugNavigation();
}, 1000);


// WebSocket connection handler
wss.on('connection', (ws) => {
    console.log('Client connected!');

    ws.send(JSON.stringify({
        type: 'status',
        speeds: robotState,
        message: 'Connected to ROS2 server'
    }));

    ws.on('message', async (message) => {
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

                case 'reset_to_start':
                    console.log('Resetting to start position:', ROBOT_START_POSITION);
    
                    broadcastToClients({
                        type: 'nav_status',
                        data: {
                            navigation: 'resetting',
                            localization: 'initializing',
                            feedback: 'initializing',
                            eta: 'N/A',
                            distance: 'N/A',
                            timeTaken: '0 s',
                            recoveries: '0'
                        }
                    });

                    // Using the correct ROS2 topic pub command format
                    const initCmd = `ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{ 
                        header: { 
                            frame_id: 'map'
                        }, 
                        pose: {
                            pose: {
                                position: {
                                    x: -2.0,
                                    y: -0.5,
                                    z: 0.0
                                },
                                orientation: {
                                    x: 0.0,
                                    y: 0.0,
                                    z: 0.0,
                                    w: 1.0
                                }
                            },
                            covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
                        }
                    }"`;

                    // Execute the command
                    exec(initCmd, (error, stdout, stderr) => {
                        if (error) {
                            console.error('Error setting initial pose:', error);
                            console.error('stderr:', stderr);
                            return;
                        }
                        
                        console.log('Initial pose command sent');
                        
                        // Update status after pose set
                        setTimeout(() => {
                            broadcastToClients({
                                type: 'nav_status',
                                data: {
                                    navigation: 'undefined',
                                    localization: 'active',
                                    feedback: 'undefined',
                                    eta: 'N/A',
                                    distance: 'N/A',
                                    timeTaken: '0 s',
                                    recoveries: '0'
                                }
                            });
                        }, 1000);
                    });
                    break;

                case 'navigate_to_target':
                    console.log('Setting navigation goal:', TARGET_POSITION);
    
                    // Reset navigation start time
                    navigationStartTime = Date.now();

                    // Set initial status
                    broadcastToClients({
                        type: 'nav_status',
                        data: {
                            navigation: 'active',
                            localization: 'active',
                            feedback: 'active',
                            eta: 'calculating...',
                            distance: 'calculating...',
                            timeTaken: '0 s',
                            recoveries: '0'
                        }
                    });

                    const navCmd = `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
                        pose: {
                            header: {
                                frame_id: 'map'
                            },
                            pose: {
                                position: {
                                    x: ${TARGET_POSITION.x},
                                    y: ${TARGET_POSITION.y},
                                    z: 0.0
                                },
                                orientation: {
                                    x: 0.0,
                                    y: 0.0,
                                    z: ${Math.sin(TARGET_POSITION.theta / 2)},
                                    w: ${Math.cos(TARGET_POSITION.theta / 2)}
                                }
                            }
                        }
                    }"`;

                    exec(navCmd, { maxBuffer: 1024 * 1024 }, (error) => {
                        if (error) {
                            console.error('Failed to set navigation goal:', error);
                            navigationStartTime = null;
                            broadcastToClients({
                                type: 'nav_status',
                                data: {
                                    navigation: 'undefined',
                                    localization: 'undefined',
                                    feedback: 'undefined',
                                    eta: 'NaN s',
                                    distance: 'N/A',
                                    timeTaken: 'NaN s',
                                    recoveries: '0'
                                }
                            });
                        } else {
                            console.log('Navigation goal set successfully');
                        }
                    });
                    break;


                case 'cancel_navigation':
                    navigationStartTime = null;  // Reset the start time when navigation is cancelled
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

    checkROS2Navigation();
    checkTopics();

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