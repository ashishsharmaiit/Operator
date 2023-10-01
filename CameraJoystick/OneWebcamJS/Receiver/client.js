var pc = null;
//new code v
let dataChannel = null;
const gamepads = {};


window.negotiate = function negotiate()  {
        pc.addTransceiver('video', {direction: 'recvonly'});
        pc.addTransceiver('audio', {direction: 'recvonly'});
    
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
            var ws = new WebSocket('ws://35.90.8.150:8082/');
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
    
    
            dataChannel.onopen = event => {
                console.log('Data channel opened');
                dataChannel.send('Hello, World!'); // Send "hello world" when the channel is opened
              };
          
            dataChannel.onmessage = event => {
                console.log('Received message:', event.data);
              };
          
            pc.ondatachannel = event => {
            console.log('Data channel received:', event.channel);
            const dataChannel = event.channel;
            
                dataChannel.onopen = event => {
                    console.log('Data channel opened');
                    dataChannel.send('Hello, World!'); // Send "hello world" when the channel is opened
                    console.log('Hello World Sent');
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

    // connect audio / video
    pc.addEventListener('track', function(evt) {
        if (evt.track.kind == 'video') {
            document.getElementById('video').srcObject = evt.streams[0];
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
