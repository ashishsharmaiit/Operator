
let currentView = 1;
window.rotationState = 0; 
const video1 = document.getElementById("video1");
const video2 = document.getElementById("video2");
let currentDotPosition = {
    x: video1.offsetWidth / 2,
    y: video1.offsetHeight / 2
};

let xMinValue, xMaxValue, yMinValue, yMaxValue;

const [video2width, video2height] = document.getElementById('video_size2').value.split('x');

var video2Scale = video2height/video2width;


export function switchView() {
    const videoContainer = document.getElementById('videoContainer');
    videoContainer.classList.remove('view1', 'view2', 'view3');
    currentView = (currentView%3)+1;
    console.log("triggering view",currentView)
    videoContainer.classList.add('view' + currentView);
}

export function toggleRotation() {
    window.rotationState = (window.rotationState + 90) % 360; // Increment by 90 and keep it between 0 and 360
    if (window.rotationState % 180 === 0) { // Landscape mode (0° or 180°)
        video2.style.transform = `rotate(${window.rotationState}deg)`;
    } else { // Portrait mode (90° or 270°)
        video2.style.transform = `rotate(${window.rotationState}deg) scale(${video2Scale})`;
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
    const popup = document.getElementById('autoModePopup');
    
    statusBar.innerText = message;
    popup.innerText = message;
    
    // Make the popup visible
    popup.classList.remove('popup-hidden');
    popup.classList.add('popup-visible');
    
    // Hide the popup after 3 seconds
    setTimeout(() => {
        popup.classList.remove('popup-visible');
        popup.classList.add('popup-hidden');
    }, 600);   
}

export function moveDot(x, y) {
    const dot = document.getElementById('dot');
    const videoDisplaySize = getVideoDisplaySize(video1);

    currentDotPosition.x += x;
    currentDotPosition.y += y;
    xMinValue = (videoDisplaySize.containerWidth-videoDisplaySize.displayWidth)/2;
    xMaxValue = (videoDisplaySize.containerWidth+videoDisplaySize.displayWidth)/2;
    yMinValue = (videoDisplaySize.containerHeight-videoDisplaySize.displayHeight)/2;
    yMaxValue = (videoDisplaySize.containerHeight+videoDisplaySize.displayHeight)/2;

    currentDotPosition.x = Math.max(xMinValue, Math.min(xMaxValue, currentDotPosition.x));
    currentDotPosition.y = Math.max(yMinValue, Math.min(yMaxValue, currentDotPosition.y));

    dot.style.left = `${currentDotPosition.x}px`;
    dot.style.top = `${currentDotPosition.y}px`;
}


export function sendDotCoordinates (dataChannel) {
    let relativeDotPosition = {};
    relativeDotPosition.x = (currentDotPosition.x-xMinValue)/(xMaxValue-xMinValue)*100
    relativeDotPosition.y = (currentDotPosition.y-yMinValue)/(yMaxValue-yMinValue)*100
    let date = new Date()
    const senddata = {
        mode: "auto",
        timestamp: date,
        axes: null,
        buttons: null,
        x: relativeDotPosition.x,
        y: relativeDotPosition.y
    };
    const json = JSON.stringify(senddata);
    dataChannel.send(json); // Send gamepad data to the other peer
}

function getVideoDisplaySize(videoElement) {
    var videoWidth = videoElement.videoWidth;
    var videoHeight = videoElement.videoHeight;

    var containerWidth = videoElement.offsetWidth;
    var containerHeight = videoElement.offsetHeight;

    var videoAspectRatio = videoWidth / videoHeight;
    var containerAspectRatio = containerWidth / containerHeight;

    let displayWidth, displayHeight;

    if (videoAspectRatio > containerAspectRatio) {
        // Video is wider compared to container - Width will be maxed to container
        displayWidth = containerWidth;
        displayHeight = containerWidth / videoAspectRatio;
    } else {
        // Video is taller compared to container - Height will be maxed to container
        displayHeight = containerHeight;
        displayWidth = containerHeight * videoAspectRatio;
    }

    return { displayWidth, displayHeight, containerWidth, containerHeight };
}
