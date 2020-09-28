// taken from http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality
var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

document.addEventListener("DOMContentLoaded", function() {
    function setInitialPose() {
        var poseTopic = new ROSLIB.Topic({
            ros : ros,
            name : '/localization/initialpose',
            messageType : 'geometry_msgs/msg/PoseWithCovarianceStamped'
        });

        const pose = new ROSLIB.Message({
            header : {
                // stamp should be ignored
                stamp : {
                    sec: 0,
                    nanosec: 0
                },
                frame_id : "map"
            },
            pose : {
                // Initial position in LGSVL
                pose : {
                    position : {
                        x: -57.463,
                        y: -41.644,
                        z: -2.01,
                    },
                    orientation : {
                        x: 0.0,
                        y: 0.0,
                        z: -0.99917,
                        w: 0.04059,
                    },
                },
                // 36 elements
                covariance : [
                    0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
                    0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                    0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                    0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                    0.0,  0.0,  0.0, 0.0, 0.0, 0.068,
                ],
            }
        });
        poseTopic.publish(pose);
        console.log('initial pose published');
    }

    var button = document.querySelector("[name='initial']");
    button.addEventListener('click', setInitialPose);

    function setGoalPose() {
        var poseTopic = new ROSLIB.Topic({
            ros : ros,
            name : '/planning/goal_pose',
            messageType : 'geometry_msgs/msg/PoseStamped'
        });

        const pose = new ROSLIB.Message({
            header : {
                // stamp should be ignored
                stamp : {
                    sec: 0,
                    nanosec: 0
                },
                frame_id : "map"
            },
            pose : {
                // 5th parking spot from the end in front of Autonomous Stuff office building
                position : {
                    x: -96.68548583984375,
                    y: 58.45323181152344,
                    z: 0.0, // TODO 0.0 is output by map but is it right in reality?
                },
                orientation : {
                    x: 0.0,
                    y: 0.0,
                    z: 0.9073245099158421,
                    w: 0.4204310094486097,
                },
            },
        });
        poseTopic.publish(pose);
        console.log('goal pose published');
    }

    button = document.querySelector("[name='park']");
    button.addEventListener('click', setGoalPose);
})
