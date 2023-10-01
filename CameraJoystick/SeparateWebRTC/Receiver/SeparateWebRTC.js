var pc1 = null;
var pc2 = null;
//new code v
let dataChannel = null;
const gamepads = {};
let keepaliveInterval = null;

window.rotationState = 0; // 0 or 90 degrees
let currentView = 1;  // starting with the first view
var videoScale;


function updateDataChannelOutput(message) {
    const outputDiv = document.getElementById('dataChannelOutput');
    const messageDiv = document.createElement('div');

    const formattedMessage = message.trim(); // Remove leading/trailing whitespace
    if (formattedMessage.includes("[ERROR]")) {
        messageDiv.style.color = "red";
    } else if (formattedMessage.includes("[WARNING]")) {
        messageDiv.style.color = "blue"; //replace with just "yellow" in case lighter shade is needed. This color code is for darker shade.
    } else {
        messageDiv.style.color = "black";
    }


    messageDiv.textContent = formattedMessage;
    outputDiv.appendChild(messageDiv);
}

function scrollToBottom() {
    var output = document.querySelector('.data-output');
    output.scrollTop = output.scrollHeight;
}

window.negotiate = function negotiate()  {
        pc1.addTransceiver('video', {direction: 'recvonly'});
        pc1.addTransceiver('audio', {direction: 'recvonly'});
    
        //new code v
        dataChannel = pc1.createDataChannel('dataChannel');
    
        return pc1.createOffer().then(function(offer) {
            return pc1.setLocalDescription(offer);
        }).then(function() {
            // wait for ICE gathering to complete
            return new Promise(function(resolve) {
                if (pc1.iceGatheringState === 'complete') {
                    resolve();
                } else {
                    function checkState() {
                        if (pc1.iceGatheringState === 'complete') {
                            pc1.removeEventListener('icegatheringstatechange', checkState);
                            resolve();
                        }
                    }
                    pc1.addEventListener('icegatheringstatechange', checkState);
                }
            });
        }).then(function() {
            var offer = pc1.localDescription;
            
            //new code starts
            var ws = new WebSocket('ws://34.221.226.79:8083/');
            console.log('sending offer')
            ws.onopen = function (event) {

                const [video1width, video1height] = document.getElementById('video_size1').value.split('x');

                videoScale = video1height/video1width;

                ws.send(JSON.stringify({
                    sdp: offer.sdp,
                    type: offer.type,
                    frame1: document.getElementById('frame1').value,
                    video_size1: document.getElementById('video_size1').value,
                    codec1: document.getElementById('codec1').value,               
                    audio_codec: document.getElementById('audio_codec').value,                        
                }));
            };
    
            ws.onmessage = function (event) {
                console.log('received message', event.data);
                let reader = new FileReader();
                reader.onload = function() {
                    let parsedMessage = JSON.parse(reader.result);
                    console.log('received answer', parsedMessage);
                    if (parsedMessage.type && parsedMessage.sdp) {
                        pc1.setRemoteDescription(new RTCSessionDescription(parsedMessage)).catch(e => console.log(e));
                    } else {
                        console.log("Invalid SDP message received.");
                    }
                }
                reader.readAsText(event.data);
            };
            
    
            ws.onerror = function (error) {
                console.log('WebSocket Error: ', error);
            };
    

            dataChannel.onerror = (error) => {
                console.log('Data Channel Error:', error);
            };
            
            dataChannel.onclose = () => {
                console.log('The Data Channel is Closed');
            };
            
          
            pc1.ondatachannel = event => {
            console.log('Data channel received:', event.channel);
            const dataChannel = event.channel;
            
                dataChannel.onopen = event => {
                    console.log('Data channel opened');
                    dataChannel.send('Hello, World!'); // Send "hello world" when the channel is opened
                    console.log('Hello World Sent');

                    let count = 0;
                    setInterval(() => {
                        count++;
                        dataChannel.send(`Hello ${count}`);
                    //    console.log(`Hello ${count} sent`);
                    }, 10000); // Every 10 seconds
            

                    setInterval(() => {
                        const gamepadList = navigator.getGamepads();
                        for (let i = 0; i < gamepadList.length; i++) {
                          const gamepad = gamepadList[i];
                          if (gamepad) {
                            if (!gamepads[gamepad.index] || gamepad.timestamp !== gamepads[gamepad.index].timestamp) {
                              console.log(`Gamepad ${i} status changed:`);
                             // console.log(gamepad);
                              gamepads[gamepad.index] = gamepad;
                              const gamepadData = {
                                id: gamepad.id,
                                timestamp: gamepad.timestamp,
                                axes: gamepad.axes,
                                buttons: gamepad.buttons.map(button => ({ pressed: button.pressed, value: button.value })),
                                connected: gamepad.connected
                              };
                              if (gamepad.buttons[2] && gamepad.buttons[2].pressed) {
                                    switchView();
                                }            
                            if (gamepad.buttons[3] && gamepad.buttons[3].pressed) {
                                toggleRotation();
                
                            }                                                        
                              const json = JSON.stringify(gamepadData);
                              dataChannel.send(json); // Send "hello world" when the channel is opened
                           //   console.log('Gamepad State Changed Message Sent', json);
                            }
                          } else if (gamepads[i]) {
                            console.log(`Gamepad ${i} disconnected`);
                            delete gamepads[i];
                          }
                        }
                      }, 100); // Every 100 milliseconds
              
                    }
    
                dataChannel.onmessage = event => {
                    console.log('Received message:', event.data);
                    updateDataChannelOutput(event.data);
                    scrollToBottom();
                    };
            
            }   
                          
    
    
        }).catch(function(e) {
            alert(e);
        });
}
    

window.negotiate2 = function negotiate2()  {
    pc2.addTransceiver('video', {direction: 'recvonly'});

    return pc2.createOffer().then(function(offer) {
        return pc2.setLocalDescription(offer);
    }).then(function() {
        // wait for ICE gathering to complete
        return new Promise(function(resolve) {
            if (pc2.iceGatheringState === 'complete') {
                resolve();
            } else {
                function checkState() {
                    if (pc2.iceGatheringState === 'complete') {
                        pc2.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                }
                pc2.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(function() {
        var offer = pc2.localDescription;
        
        //new code starts
        var ws2 = new WebSocket('ws://34.221.226.79:8084/');
        console.log('sending offer')
        ws2.onopen = function (event) {


            ws2.send(JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
                frame2: document.getElementById('frame2').value,
                video_size2: document.getElementById('video_size2').value,
                codec2: document.getElementById('codec2').value,      
            }));
        };

        ws2.onmessage = function (event) {
            console.log('received message', event.data);
            let reader = new FileReader();
            reader.onload = function() {
                let parsedMessage = JSON.parse(reader.result);
                console.log('received answer', parsedMessage);
                if (parsedMessage.type && parsedMessage.sdp) {
                    pc2.setRemoteDescription(new RTCSessionDescription(parsedMessage)).catch(e => console.log(e));
                } else {
                    console.log("Invalid SDP message received.");
                }
            }
            reader.readAsText(event.data);
        };
        

        ws2.onerror = function (error) {
            console.log('WebSocket Error: ', error);
        };



    }).catch(function(e) {
        alert(e);
    });
}

window.start = function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    config.iceServers = [{urls: ['stun:stun.l.google.com:19302']}];
    pc1 = new RTCPeerConnection(config);

    pc1.oniceconnectionstatechange = function(event) {
        if (pc1.iceConnectionState === 'failed' || pc1.iceConnectionState === 'disconnected') {
            console.log('Connection failed. Restarting...Current connection state: ',pc1.iceConnectionState);
            var retryInterval = setInterval(() => {
                console.log('Trying to restart connection... Current connection state: ',pc1.iceConnectionState);
                stop();  // Call your stop function to close the current connection
                start();  // Call your start function to initiate a new connection
                if (pc1.iceConnectionState !== 'failed' && pc1.iceConnectionState !== 'disconnected') {
                    console.log('Connection restarted successfully. Current connection state: ',pc1.iceConnectionState);
                    clearInterval(retryInterval);
                }
            }, 5000); // Try restarting every 5 seconds
        }
    };


    let videoContainer = document.getElementById('videoContainer');
    let videoCount = 1; // start with 1 since we already have video1 in the HTML
    
    // ...
    
    pc1.addEventListener('track', function(evt) {
        if (evt.track.kind == 'video') {
            // Create a new MediaStream for the incoming track
            let newStream = new MediaStream();
            newStream.addTrack(evt.track);
    
            // Use existing video elements if the videoCount is less than or equal to 2
            let videoElem;
            if (videoCount <= 2) {
                videoElem = document.getElementById('video' + videoCount);
            } else {
                // Create a new video element dynamically for any additional incoming tracks
                videoElem = document.createElement('video');
                videoElem.id = 'video' + videoCount;
                videoElem.autoplay = true;
                videoElem.playsinline = true;
                videoContainer.appendChild(videoElem);
            }
            
            videoElem.srcObject = newStream;
    
            // Increment the video counter
            videoCount++;
        } else {
            document.getElementById('audio').srcObject = evt.streams[0];
        }
    });
    
   
    
    document.getElementById('start').style.display = 'none';
    negotiate();
    document.getElementById('stop').style.display = 'inline-block';
}

window.start2 = function start2() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    config.iceServers = [{urls: ['stun:stun.l.google.com:19302']}];
    pc2 = new RTCPeerConnection(config);

    pc2.oniceconnectionstatechange = function(event) {
        if (pc2.iceConnectionState === 'failed' || pc2.iceConnectionState === 'disconnected') {
            console.log('Connection failed. Restarting...Current connection state: ',pc2.iceConnectionState);
            var retryInterval = setInterval(() => {
                console.log('Trying to restart connection... Current connection state: ',pc2.iceConnectionState);
                stop();  // Call your stop function to close the current connection
                start();  // Call your start function to initiate a new connection
                if (pc2.iceConnectionState !== 'failed' && pc2.iceConnectionState !== 'disconnected') {
                    console.log('Connection restarted successfully. Current connection state: ',pc2.iceConnectionState);
                    clearInterval(retryInterval);
                }
            }, 5000); // Try restarting every 5 seconds
        }
    };
    
    // ...
    
    pc2.addEventListener('track', function(evt) {
        if (evt.track.kind == 'video') {
            // Create a new MediaStream for the incoming track
            let newStream = new MediaStream();
            newStream.addTrack(evt.track);
    
            // Use existing video elements if the videoCount is less than or equal to 2
            videoElem = document.getElementById('video2');            
            videoElem.srcObject = newStream;
            } else {
        }
    });
       
    negotiate2();
}


function switchView() {
    const videoContainer = document.getElementById('videoContainer');
    videoContainer.classList.remove('view1', 'view2', 'view3','view1-90','view2-90','view3-90');
    currentView = (currentView%3)+1;
    console.log("triggering view",currentView)
    if (window.rotationState % 180 === 0) { // Landscape mode (0° or 180°)
        videoContainer.classList.add('view' + currentView);
    } else {
        videoContainer.classList.add('view' + currentView + '-90');
    }
}

window.toggleRotation = function() {
    const video1 = document.getElementById("video1");
    const videoContainer = document.getElementById('videoContainer');
    window.rotationState = (window.rotationState + 90) % 360; // Increment by 90 and keep it between 0 and 360
    if (window.rotationState % 180 === 0) { // Landscape mode (0° or 180°)
        video1.style.transform = `rotate(${window.rotationState}deg)`;
    } else { // Portrait mode (90° or 270°)
        video1.style.transform = `rotate(${window.rotationState}deg) scale(${videoScale})`;
    }
};





window.stop = function stop() {
    document.getElementById('stop').style.display = 'none';

    // close peer connection
    setTimeout(function() {
        pc1.close();
        pc2.close();
    }, 500);
}

window.addEventListener('gamepadconnected', event => {
    const gamepad = event.gamepad;
    console.log(`Gamepad connected: ${gamepad}`);
    gamepads[gamepad.index] = gamepad;
});

window.addEventListener('gamepaddisconnected', event => {
    const gamepad = event.gamepad;
    console.log(`Gamepad disconnected: ${gamepad}`);
    delete gamepads[gamepad.index];
});
