
let currentView = 1;
var videoScale;
window.rotationState = 0; 
const video1 = document.getElementById("video1");
let currentDotPosition = {
    x: video1.offsetWidth / 2,
    y: video1.offsetHeight / 2
};

const [video1width, video1height] = document.getElementById('video_size1').value.split('x');

videoScale = video1height/video1width;


export function switchView() {
    const videoContainer = document.getElementById('videoContainer');
    videoContainer.classList.remove('view1', 'view2', 'view3');
    currentView = (currentView%3)+1;
    console.log("triggering view",currentView)
    videoContainer.classList.add('view' + currentView);
}

export function toggleRotation() {
    const videoContainer = document.getElementById('videoContainer');
    window.rotationState = (window.rotationState + 90) % 360; // Increment by 90 and keep it between 0 and 360
    if (window.rotationState % 180 === 0) { // Landscape mode (0° or 180°)
        video1.style.transform = `rotate(${window.rotationState}deg)`;
    } else { // Portrait mode (90° or 270°)
        video1.style.transform = `rotate(${window.rotationState}deg) scale(${videoScale})`;
    }
}

export function getFormDataAsJSON(offer) {
    return JSON.stringify({
        sdp: offer.sdp,
        type: offer.type,
        frame1: document.getElementById('frame1').value,
        video_size1: document.getElementById('video_size1').value,
        codec1: document.getElementById('codec1').value,               
        frame2: document.getElementById('frame2').value,
        video_size2: document.getElementById('video_size2').value,
        codec2: document.getElementById('codec2').value,      
        audio_codec: document.getElementById('audio_codec').value,                        
    })
}


export function updateDataChannelOutput(message) {
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

export function scrollToBottom() {
    var output = document.querySelector('.data-output');
    output.scrollTop = output.scrollHeight;
}


export function modeSet(message) {
    const statusBar = document.getElementById("statusBar");
    statusBar.innerText = message;
}

export function moveDot (x,y) {
    const dot = document.getElementById('dot');
    
    currentDotPosition.x += x;
    currentDotPosition.y += y;

    currentDotPosition.x = Math.max(0, Math.min(video1.offsetWidth, currentDotPosition.x));
    currentDotPosition.y = Math.max(0, Math.min(video1.offsetHeight, currentDotPosition.y));
   
    dot.style.left = `${currentDotPosition.x}px`;
    dot.style.top = `${currentDotPosition.y}px`;

}

export function sendDotCoordinates (dataChannel) {
    const json = JSON.stringify(currentDotPosition);
    dataChannel.send(json); // Send gamepad data to the other peer
}