var pc = null;
//new code v
let dataChannel = null;
const gamepads = {};
let keepaliveInterval = null;


let videoCount = 0;  // Counter to handle multiple video elements


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
            var ws = new WebSocket('ws://54.218.1.229:8083/');
            console.log('sending offer',JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
            }))
            ws.onopen = function (event) {
                ws.send(JSON.stringify({
                    sdp: offer.sdp,
                    type: offer.type,
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
                        console.log(`Hello ${count} sent`);
                    }, 10000); // Every 10 seconds
            

                    setInterval(() => {
                        const gamepadList = navigator.getGamepads();
                        for (let i = 0; i < gamepadList.length; i++) {
                          const gamepad = gamepadList[i];
                          if (gamepad) {
                            if (!gamepads[gamepad.index] || gamepad.timestamp !== gamepads[gamepad.index].timestamp) {
                              console.log(`Gamepad ${i} status changed:`);
                              console.log(gamepad);
                              gamepads[gamepad.index] = gamepad;
                              const gamepadData = {
                                id: gamepad.id,
                                timestamp: gamepad.timestamp,
                                axes: gamepad.axes,
                                buttons: gamepad.buttons.map(button => ({ pressed: button.pressed, value: button.value })),
                                connected: gamepad.connected
                              };
                              const json = JSON.stringify(gamepadData);
                              dataChannel.send(json); // Send "hello world" when the channel is opened
                              console.log('Gamepad State Changed Message Sent', json);
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

    if (document.getElementById('use-stun').checked) {
        config.iceServers = [{urls: ['stun:stun.l.google.com:19302']}];
    }

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


    // connect audio / video
    var videoElement = null; // Declare a variable to store the video element

    pc.addEventListener('track', function(evt) {
        if (evt.track.kind == 'video') {
            // Create a new MediaStream for the incoming track
            let newStream = new MediaStream();
            newStream.addTrack(evt.track);
    
            // Create a new video element dynamically for the incoming track
            let newVideoElem = document.createElement('video');
            newVideoElem.id = 'video' + videoCount;
            newVideoElem.srcObject = newStream;
            newVideoElem.autoplay = true;
            newVideoElem.playsinline = true;
    
            // Add the new video element to the document
            document.body.appendChild(newVideoElem);
    
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
