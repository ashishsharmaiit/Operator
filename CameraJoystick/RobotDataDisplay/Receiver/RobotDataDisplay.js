var pc = null;
//new code v
let dataChannel = null;
const gamepads = {};
let keepaliveInterval = null;

window.rotationState = 0; // 0 or 90 degrees
let currentView = 1;  // starting with the first view


function updateDataChannelOutput(message) {
    const outputDiv = document.getElementById('dataChannelOutput');
    const messageDiv = document.createElement('div');
    messageDiv.textContent = message;
    outputDiv.appendChild(messageDiv);
}

function scrollToBottom() {
    var output = document.querySelector('.data-output');
    output.scrollTop = output.scrollHeight;
}

window.negotiate = function negotiate()  {
        pc.addTransceiver('video', {direction: 'recvonly'});
        pc.addTransceiver('audio', {direction: 'recvonly'});
        pc.addTransceiver('video', {direction: 'recvonly'});
    
        //new code v
        dataChannel = pc.createDataChannel('dataChannel');
    
        return pc.createOffer().then(function(offer) {
            return pc.setLocalDescription(offer);
        }).then(function() {
            // wait for ICE gathering to complete
            return new Promise(function(resolve) {
                if (pc.iceGatheringState === 'complete') {
                    resolve();
                } else {
                    function checkState() {
                        if (pc.iceGatheringState === 'complete') {
                            pc.removeEventListener('icegatheringstatechange', checkState);
                            resolve();
                        }
                    }
                    pc.addEventListener('icegatheringstatechange', checkState);
                }
            });
        }).then(function() {
            var offer = pc.localDescription;
            
            //new code starts
            var ws = new WebSocket('ws://54.218.1.229:8084/');
            console.log('sending offer')
            ws.onopen = function (event) {
                ws.send(JSON.stringify({
                    sdp: offer.sdp,
                    type: offer.type,
                    frame1: document.getElementById('frame1').value,
                    video_size1: document.getElementById('video_size1').value,
                    codec1: document.getElementById('codec1').value,               
                    frame2: document.getElementById('frame2').value,
                    video_size2: document.getElementById('video_size2').value,
                    codec2: document.getElementById('codec2').value,      
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
                        pc.setRemoteDescription(new RTCSessionDescription(parsedMessage)).catch(e => console.log(e));
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
            
          
            pc.ondatachannel = event => {
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
    


window.start = function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    config.iceServers = [{urls: ['stun:stun.l.google.com:19302']}];
    pc = new RTCPeerConnection(config);

    pc.oniceconnectionstatechange = function(event) {
        if (pc.iceConnectionState === 'failed' || pc.iceConnectionState === 'disconnected') {
            console.log('Connection failed. Restarting...Current connection state: ',pc.iceConnectionState);
            var retryInterval = setInterval(() => {
                console.log('Trying to restart connection... Current connection state: ',pc.iceConnectionState);
                stop();  // Call your stop function to close the current connection
                start();  // Call your start function to initiate a new connection
                if (pc.iceConnectionState !== 'failed' && pc.iceConnectionState !== 'disconnected') {
                    console.log('Connection restarted successfully. Current connection state: ',pc.iceConnectionState);
                    clearInterval(retryInterval);
                }
            }, 5000); // Try restarting every 5 seconds
        }
    };


    let videoContainer = document.getElementById('videoContainer');
    let videoCount = 1; // start with 1 since we already have video1 in the HTML
    
    // ...
    
    pc.addEventListener('track', function(evt) {
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



function switchView() {
    const videoContainer = document.getElementById('videoContainer');
    videoContainer.classList.remove('view1', 'view2', 'view3');
    currentView = (currentView%3)+1;
    console.log("triggering view",currentView)
    videoContainer.classList.add('view' + currentView);
    }


window.toggleRotation = function() {
    const video1 = document.getElementById("video1");
    window.rotationState = (window.rotationState + 90) % 360; // Increment by 90 and keep it between 0 and 360

    video1.style.transform = `rotate(${window.rotationState}deg)`;

    if (window.rotationState % 180 === 0) { // Landscape mode (0° or 180°)
        video1.style.width = "";
        video1.style.height = "";
    } else { // Portrait mode (90° or 270°)
        const videoWidth = video1.offsetWidth;
        const videoHeight = video1.offsetHeight;

        video1.style.width = `${videoHeight}px`;
        video1.style.height = `${videoWidth}px`;
    }
};





window.stop = function stop() {
    document.getElementById('stop').style.display = 'none';

    // close peer connection
    setTimeout(function() {
        pc.close();
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
