const char INDEX_HTML[] PROGMEM = R"=====(

<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, maximum-scale=1.0, user-scalable=no">
    <title>Motor Control Interface</title>
    <style>
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: flex-start;
            min-height: 100vh;
            margin: 0;
            font-family: Arial, sans-serif;
            padding: 20px;
            background: #f5f5f5;
        }

        #speedDisplay {
            text-align: center;
            font-size: 1.2rem;
            margin: 20px 0;
            padding: 15px;
            background: white;
            border-radius: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            width: 100%;
            max-width: 400px;
        }

        #controlContainer {
            display: flex;
            flex-direction: column;
            gap: 30px;
            width: 100%;
            max-width: 400px;
            margin-top: 20px;
        }

        .sliderContainer {
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            width: 100%;
        }

        .sliderContainer label {
            display: block;
            margin-bottom: 15px;
            font-weight: bold;
            color: #333;
            font-size: 1.1rem;
            text-align: center;
        }

        .verticalSliderWrapper {
            height: 250px;
            position: relative;
            display: flex;
            justify-content: center;
            align-items: center;
        }

        /* Vertical slider styles */
        #verticalSlider {
            -webkit-appearance: none;
            appearance: none;
            width: 250px;
            height: 12px;
            background: #e0e0e0;
            outline: none;
            transform: rotate(-90deg);
            border-radius: 6px;
        }

        #verticalSlider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 28px;
            height: 28px;
            background: #007BFF;
            cursor: pointer;
            border-radius: 50%;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }

        /* Horizontal slider container styles */
        .horizontalSliderWrapper {
            display: flex;
            justify-content: center;
            align-items: center;
            width: 100%;
            padding: 20px 0;
        }

        /* Horizontal slider styles */
        #horizontalSlider {
            -webkit-appearance: none;
            appearance: none;
            width: 100%;
            height: 12px;
            background: #e0e0e0;
            outline: none;
            border-radius: 6px;
        }

        #horizontalSlider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 28px;
            height: 28px;
            background: #007BFF;
            cursor: pointer;
            border-radius: 50%;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }

        /* Value displays */
        .value-display {
            font-size: 1.1rem;
            margin: 5px 0;
        }

        /* New PID Controller container styles */
        #pidControlContainer {
            margin-top: 40px;
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            width: auto;
            /* overflow-wrap: break-word; */
            text-wrap: stable;
        }

        #pidControlContainer label {
            font-weight: bold;
            color: #333;
            font-size: 1.1rem;
            margin-bottom: 10px;
        }

        #pidControlContainer input[type="number"] {
            width: 80px;
            padding: 10px;
            font-size: 1rem;
            margin: 5px;
            border-radius: 5px;
            border: 1px solid #ddd;
        }

        #pidControlContainer input[type="checkbox"] {
            margin: 10px;
        }

        #pidControlContainer button {
            background: #007BFF;
            color: white;
            font-size: 1.1rem;
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            transition: background 0.3s;
        }

        #pidControlContainer button:hover {
            background: #0056b3;
        }

        #resetButton {
            margin-top: 30px;
            background: #f73636;
            color: white;
            font-size: 1.1rem;
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            transition: background 0.3s;
        }

        #resetButton:hover {
            background: #bc1c1c;
        }
        
        #resetButton:active {
            background: #9f1212;
        }

        #resetDirectionButton {
            margin-top: 30px;
            background: #007BFF;
            color: white;
            font-size: 1.1rem;
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            transition: background 0.3s;
        }

        /* Hover effect for reset button */
        #resetDirectionButton:hover {
            background: #0056b3;
        }

        /* Active (pressed) effect for reset button */
        #resetDirectionButton:active {
            background: #004080;
        }

        #buttonContainer {
            display: flex;
            gap: 10px;
            margin-top: 20px;
        }

        @media (min-width: 768px) {
            #controlContainer {
                flex-direction: row;
                max-width: 800px;
                gap: 40px;
            }

            .sliderContainer {
                flex: 1;
                display: flex;
                flex-direction: column;
            }

            .horizontalSliderWrapper {
                flex: 1;
                display: flex;
                align-items: center;
            }

            #horizontalSlider {
                width: 80%;
            }

            #speedDisplay {
                font-size: 1.5rem;
                max-width: 800px;
            }
        }
    </style>
</head>
<body>
    <div id="speedDisplay">
        <div class="value-display">Speed: <span id="speedValue">0</span>% / <span id="rpmValue">0</span> RPM</div>
        <div class="value-display">Direction: <span id="directionValue">Neutral</span></div>
        <div class="value-display">Forward/Backward: <span id="forwardBackwardValue">Neutral</span></div>
    </div>

    <div id="controlContainer">
        <div class="sliderContainer">
            <label for="verticalSlider">Speed</label>
            <div class="verticalSliderWrapper">
                <input type="range" id="verticalSlider" min="-100" max="100" value="0">
            </div>
        </div>

        <div class="sliderContainer">
            <label for="horizontalSlider">Direction</label>
            <div class="horizontalSliderWrapper">
                <input type="range" id="horizontalSlider" min="-50" max="50" value="0">
            </div>
        </div>
    </div>

    <div id="buttonContainer">
        <button id="resetButton">Reset</button>
        <button id="resetDirectionButton">Neutral</button>
    </div>
    
    <!-- PID Controller Form -->
    <div id="pidControlContainer">
        <label for="kpInput">Kp:</label>
        <input type="number" id="kpInput" value="1.0" step="0.1">

        <label for="kiInput">Ki:</label>
        <input type="number" id="kiInput" value="0.1" step="0.1">

        <label for="kdInput">Kd:</label>
        <input type="number" id="kdInput" value="0.0" step="0.1">

        <label for="enableControl">
            <input type="checkbox" id="enableControl"> Enable Control
        </label>

        <button id="sendButton">Send</button>
    </div>
    

    <script>
        const speedValueElement = document.getElementById("speedValue");
        const rpmValueElement = document.getElementById("rpmValue");
        const directionValueElement = document.getElementById("directionValue");
        const forwardBackwardValueElement = document.getElementById("forwardBackwardValue");
        const verticalSlider = document.getElementById("verticalSlider");
        const horizontalSlider = document.getElementById("horizontalSlider");

        const kpInput = document.getElementById("kpInput");
        const kiInput = document.getElementById("kiInput");
        const kdInput = document.getElementById("kdInput");
        const enableControl = document.getElementById("enableControl");
        const sendButton = document.getElementById("sendButton");
        const resetButton = document.getElementById("resetButton");
        const resetDirectionButton = document.getElementById("resetDirectionButton");

        const maxRPM = 110;

        verticalSlider.addEventListener("input", () => {
            const speedPercent = verticalSlider.value;
            const rpm = calculateRPM(speedPercent);

            speedValueElement.innerText = Math.abs(speedPercent);
            rpmValueElement.innerText = Math.abs(rpm);
            directionValueElement.innerText = getDirection(horizontalSlider.value);
            forwardBackwardValueElement.innerText = speedPercent >= 0 ? "Forward" : "Backward";
            updateMotor(speedPercent, horizontalSlider.value);
        });

        horizontalSlider.addEventListener("input", () => {
            directionValueElement.innerText = getDirection(horizontalSlider.value);
            updateMotor(verticalSlider.value, horizontalSlider.value);
        });

        sendButton.addEventListener("click", () => {
            const Kp = parseFloat(kpInput.value);
            const Ki = parseFloat(kiInput.value);
            const Kd = parseFloat(kdInput.value);
            const controlEnabled = enableControl.checked;

            // Send the PID values and control status to the server
            console.log("PID Values:", Kp, Ki, Kd, "Control Enabled:", controlEnabled);
            // You can now send these values to the server as needed
            updatePID(Kp, Ki, Kd, controlEnabled);
        });
    

        resetButton.addEventListener("click", () => {
            // Set both sliders to zero
            verticalSlider.value = 0;
            horizontalSlider.value = 0;

            // Update displayed values
            speedValueElement.innerText = 0;
            rpmValueElement.innerText = 0;
            directionValueElement.innerText = "Neutral";
            forwardBackwardValueElement.innerText = "Neutral";

            // Send update to server to reset motor settings
            updateMotor(0, 0);
        });
        
        
        // Add event listener for the reset direction button
        resetDirectionButton.addEventListener("click", () => {
            // Set horizontal slider to zero
            horizontalSlider.value = 0;

            // Update displayed values
            directionValueElement.innerText = "Neutral";

            // Send update to server to reset direction
            updateMotor(verticalSlider.value, 0);
        });

        // Add keyboard controls
        document.addEventListener('keydown', function(event) {
            const key = event.key.toLowerCase();
            switch(key) {
                case 'w':
                case 'arrowup':
                    event.preventDefault();
                    verticalSlider.value = Math.min(parseInt(verticalSlider.value) + 5, 100);
                    verticalSlider.dispatchEvent(new Event('input'));
                    break;
                case 's':
                case 'arrowdown':
                    event.preventDefault();
                    verticalSlider.value = Math.max(parseInt(verticalSlider.value) - 5, -100);
                    verticalSlider.dispatchEvent(new Event('input'));
                    break;
                case 'a':
                case 'arrowleft':
                    event.preventDefault();
                    horizontalSlider.value = Math.max(parseInt(horizontalSlider.value) - 5, -50);
                    horizontalSlider.dispatchEvent(new Event('input'));
                    break;
                case 'd':
                case 'arrowright':
                    event.preventDefault();
                    horizontalSlider.value = Math.min(parseInt(horizontalSlider.value) + 5, 50);
                    horizontalSlider.dispatchEvent(new Event('input'));
                    break;
                case 'f':
                    event.preventDefault();
                    verticalSlider.value = 0;
                    horizontalSlider.value = 0;
                    speedValueElement.innerText = 0;
                    rpmValueElement.innerText = 0;
                    directionValueElement.innerText = "Neutral";
                    forwardBackwardValueElement.innerText = "Neutral";
                    updateMotor(0, 0);
                    break;
                case 'q':
                    event.preventDefault();
                    horizontalSlider.value = 0;
                    directionValueElement.innerText = "Neutral";
                    const currentSpeed = verticalSlider.value;
                    updateMotor(currentSpeed, 0);
                    break;
            }
        });

        function calculateRPM(percent) {
            return Math.round((percent / 100.0) * maxRPM);
        }

        function getDirection(turnRate) {
            if (turnRate === 0) {
                return "Straight";
            } else if (turnRate > 0) {
                return `Right ${turnRate} %`;
            } else {
                return `Left ${-turnRate} %`;
            }
        }
 
        function updatePID(Kp, Ki, Kd, enabled) {
        	const queryString = `${window.location.origin}/setPID?` +
                `kp=${Kp}` +
                `&ki=${Ki}` +
                `&kd=${Kd}`+
                `&enabled=${enabled == true ? 1 : 0}`;
             console.log(queryString);

            fetch(queryString)
                .then(response => response.text())
                .then(data => {
                    console.log('PID Parameters:', data);
                })
                .catch(error => {
                    console.error('Error setting PID:', error);
                });
        }

        function updateMotor(speed, turnRate) {
            const queryString = `${window.location.origin}/setMotor?` +
                `speed=${Math.abs(speed)}` +
                `&forwardBackward=${speed >= 0 ? "Forward" : "Backward"}` +
                `&turnRate=${turnRate}`;

            fetch(queryString)
                .then(response => response.text())
                .then(data => {
                    console.log('Motor response:', data);
                })
                .catch(error => {
                    console.error('Error setting motor:', error);
                });
        }
    </script>
</body>
</html>

)=====";
