import { startGamepadUpdates } from './gamepadHandler.js';
import * as UI from './UI.js';

var pc = null;
let dataChannel = null;


export function negotiate() {
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
        var ws = new WebSocket('ws://34.221.226.79:8084/');
        console.log('sending offer')
        ws.onopen = function (event) {
            ws.send(UI.getFormDataAsJSON(offer));
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
        

                startGamepadUpdates(dataChannel);           
                }

            dataChannel.onmessage = event => {
                console.log('Received message:', event.data);
                UI.updateDataChannelOutput(event.data);
                UI.scrollToBottom();
                };
        
        }   
                      


    }).catch(function(e) {
        alert(e);
    });
}

export function start() {
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



export function stop() {
    document.getElementById('stop').style.display = 'none';

    // close peer connection
    setTimeout(function() {
        pc.close();
    }, 500);
}


window.updateDataChannelOutput = UI.updateDataChannelOutput;
window.scrollToBottom = UI.scrollToBottom;
window.negotiate = negotiate;
window.start = start;
window.toggleRotation = UI.toggleRotation;
window.stop = stop;
window.getFormDataAsJSON = UI.getFormDataAsJSON;
