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
    /*
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

    var initialButton = document.querySelector("[name='initial']");
    initialButton.addEventListener('click', setInitialPose);
    */

    var goalTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/planning/goal_pose',
        messageType : 'geometry_msgs/msg/PoseStamped'
    });

    // in parsing the pose, 0.0 get converted to an integer leading to problems in publishing the
    // message. So choose something close enough to zero that preserves the float type
    const floatNearZero = 1e-16;

    function setGoalPose() {
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
                    x: -97.62905883789062,
                    y: 59.871952056884766,
                    z: floatNearZero, // planning is done in 2D, so z irrelevant
                },
                // park in reverse
                orientation : {
                    x: floatNearZero,
                    y: floatNearZero,
                    z: 0.42534109950065613,
                    w: -0.9050331115722656,
                },
            },
        });
        goalTopic.publish(pose);
        console.log('goal pose published');
    }

    var parkButton = document.querySelector("[name='park']");
    parkButton.addEventListener('click', setGoalPose);

    function setReturnPose() {
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
                // on lane in front of Autonomous Stuff office building
                position : {
                    x: -26.73210906982422,
                    y: 108.79566192626953,
                    z: 0.0,
                },
                orientation : {
                    x: 0.0,
                    y: 0.0,
                    z: 0.3421307048194666,
                    w: 0.9396523723269872,
                },
            },
        });
        goalTopic.publish(pose);
        console.log('return pose published');
    }

    var returnButton = document.querySelector("[name='return']");
    returnButton.addEventListener('click', setReturnPose);
})
