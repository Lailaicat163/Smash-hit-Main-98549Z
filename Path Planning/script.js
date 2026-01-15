$(document).ready(function() {
const $fieldSelect = $('#field_select');
const $fields = $('.field');
const $removePointsButton = $('#remove_button');
const $addPointButton = $('#add_button');
const $pointsList = $('#points_list');
let points = [];
let pointCounter = 0;
let controlPoints = [];
let draggingPoint = null;

class Point {
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }
}

function quinticBezier(t, p0, p1, p2, p3, p4, p5) {
  const u = 1 - t;
  const u_squared = u * u;
  const u_cubed = u_squared * u;
  const u_quartic = u_cubed * u;
  const u_quintic = u_quartic * u;

  const t_squared = t * t;
  const t_cubed = t_squared * t;
  const t_quartic = t_cubed * t;
  const t_quintic = t_quartic * t;

  return {
    x:
      u_quintic * p0.x +
      5 * u_quartic * t * p1.x +
      10 * u_cubed * t_squared * p2.x +
      10 * u_squared * t_cubed * p3.x +
      5 * u * t_quartic * p4.x +
      t_quintic * p5.x,

    y:
      u_quintic * p0.y +
      5 * u_quartic * t * p1.y +
      10 * u_cubed * t_squared * p2.y +
      10 * u_squared * t_cubed * p3.y +
      5 * u * t_quartic * p4.y +
      t_quintic * p5.y
  };
}

function quinticBezierDerivative(t, p0, p1, p2, p3, p4, p5) {
    const u = 1 - t;
    const u_squared = u * u;
    const u_cubed = u_squared * u;
    const u_quartic = u_cubed * u;
    
    const t_squared = t * t;
    const t_cubed = t_squared * t;
    const t_quartic = t_cubed * t;
    
    return {
        x: -5 * u_quartic * p0.x +
            5 * u_quartic * p1.x - 20 * u_cubed * t * p1.x +
            20 * u_cubed * t * p2.x - 30 * u_squared * t_squared * p2.x +
            30 * u_squared * t_squared * p3.x - 20 * u * t_cubed * p3.x +
            20 * u * t_cubed * p4.x - 5 * t_quartic * p4.x +
            5 * t_quartic * p5.x,
            
        y: -5 * u_quartic * p0.y +
            5 * u_quartic * p1.y - 20 * u_cubed * t * p1.y +
            20 * u_cubed * t * p2.y - 30 * u_squared * t_squared * p2.y +
            30 * u_squared * t_squared * p3.y - 20 * u * t_cubed * p3.y +
            20 * u * t_cubed * p4.y - 5 * t_quartic * p4.y +
            5 * t_quartic * p5.y
    };
}

function quinticBezierSecondDerivative(t, p0, p1, p2, p3, p4, p5) {
    const u = 1 - t;
    const u_squared = u * u;
    const u_cubed = u_squared * u;
    
    const t_squared = t * t;
    const t_cubed = t_squared * t;
    
    return {
        x: 20 * u_cubed * p0.x - 40 * u_cubed * p1.x +
            60 * u_squared * t * p1.x + 20 * u_cubed * p2.x -
            120 * u_squared * t * p2.x + 60 * u * t_squared * p2.x +
            60 * u_squared * t * p3.x - 120 * u * t_squared * p3.x +
            20 * t_cubed * p3.x + 60 * u * t_squared * p4.x -
            40 * t_cubed * p4.x + 20 * t_cubed * p5.x,
            
        y: 20 * u_cubed * p0.y - 40 * u_cubed * p1.y +
            60 * u_squared * t * p1.y + 20 * u_cubed * p2.y -
            120 * u_squared * t * p2.y + 60 * u * t_squared * p2.y +
            60 * u_squared * t * p3.y - 120 * u * t_squared * p3.y +
            20 * t_cubed * p3.y + 60 * u * t_squared * p4.y -
            40 * t_cubed * p4.y + 20 * t_cubed * p5.y
    };
}

function quinticBezierThirdDerivative(t, p0, p1, p2, p3, p4, p5) {
    const u = 1 - t;
    const u_squared = u * u;
    const t_squared = t * t;
    
    return {
        x: -60 * u_squared * p0.x + 180 * u_squared * p1.x -
            240 * u * t * p1.x - 180 * u_squared * p2.x +
            360 * u * t * p2.x - 120 * t_squared * p2.x -
            120 * u * t * p3.x + 360 * t_squared * p3.x -
            240 * u * t * p4.x - 180 * t_squared * p4.x +
            180 * t_squared * p5.x + 60 * t_squared * p5.x,
            
        y: -60 * u_squared * p0.y + 180 * u_squared * p1.y -
            240 * u * t * p1.y - 180 * u_squared * p2.y +
            360 * u * t * p2.y - 120 * t_squared * p2.y -
            120 * u * t * p3.y + 360 * t_squared * p3.y -
            240 * u * t * p4.y - 180 * t_squared * p4.y +
            180 * t_squared * p5.y + 60 * t_squared * p5.y
    };
}

function calculatePathProperties(t, p0, p1, p2, p3, p4, p5, dt) {
    const position = quinticBezier(t, p0, p1, p2, p3, p4, p5);
    const velocity = quinticBezierDerivative(t, p0, p1, p2, p3, p4, p5);
    const acceleration = quinticBezierSecondDerivative(t, p0, p1, p2, p3, p4, p5);
    const jerk = quinticBezierThirdDerivative(t, p0, p1, p2, p3, p4, p5);
    
    const linearVelocity = Math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    const heading = Math.atan2(velocity.y, velocity.x);
    const curvature = (velocity.x * acceleration.y - velocity.y * acceleration.x) / 
                      Math.pow(velocity.x * velocity.x + velocity.y * velocity.y, 1.5);
    
    const angularVelocity = linearVelocity * curvature;
    const linearAcceleration = Math.sqrt(acceleration.x * acceleration.x + 
                                         acceleration.y * acceleration.y);
    
    const tangentialAcceleration = (velocity.x * acceleration.x + 
                                    velocity.y * acceleration.y) / linearVelocity;
    
    const centripetalAcceleration = linearVelocity * linearVelocity * Math.abs(curvature);
    const linearJerk = Math.sqrt(jerk.x * jerk.x + jerk.y * jerk.y);
    
    return {
        position: position,
        linearVelocity: linearVelocity,
        heading: heading,
        curvature: curvature,
        angularVelocity: angularVelocity,
        linearAcceleration: linearAcceleration,
        tangentialAcceleration: tangentialAcceleration,
        centripetalAcceleration: centripetalAcceleration,
        linearJerk: linearJerk,
        velocity: velocity,
        acceleration: acceleration,
        jerk: jerk
    };
}

function generateQuinticPathProfile() {
    const pathProfile = [];
    
    if (points.length < 2) return pathProfile;
    
    for (let segment = 0; segment < points.length - 1; segment++) {
        const p0 = points[segment];
        const p5 = points[segment + 1];
        const controls = controlPoints[segment];
        
        if (!controls || controls.length !== 4) continue;
        
        const p1 = controls[0];
        const p2 = controls[1];
        const p3 = controls[2];
        const p4 = controls[3];
        
        const segmentMaxVel = p5.maxVelocity || 100;
        const segmentMaxAccel = p5.maxAcceleration || 150;
        
        const numPoints = 100;
        const dt = 1.0 / numPoints;
        
        for (let i = 0; i <= numPoints; i++) {
            const t = i / numPoints;
            
            const props = calculatePathProperties(t, p0, p1, p2, p3, p4, p5, dt);
            
            if (!props.linearVelocity || isNaN(props.linearVelocity)) continue;
            
            const curvatureVelocity = segmentMaxVel / (1 + Math.abs(props.curvature) * 30);
            const targetVelocity = Math.min(segmentMaxVel, curvatureVelocity);
            const velocityScale = targetVelocity / (props.linearVelocity || 1);
            
            pathProfile.push({
                x: props.position.x,
                y: props.position.y,
                heading: props.heading * 180 / Math.PI,
                linearVelocity: targetVelocity,
                angularVelocity: props.angularVelocity * velocityScale,
                curvature: props.curvature,
                acceleration: Math.min(segmentMaxAccel, Math.abs(props.tangentialAcceleration * velocityScale)),
                centripetalAccel: props.centripetalAcceleration,
                jerk: props.linearJerk * velocityScale,
                segment: segment,
                frame: pathProfile.length
            });
        }
    }
    
    return pathProfile;
}

const FIELD_WIDTH_INCHES = 144;
const FIELD_HEIGHT_INCHES = 144;

function getActiveField() {
    const $active = $('.field.active');
    return $active;
}

function showField() {
    $fields.removeClass('active');
    const selectedFieldId = $fieldSelect.val();
    $('#' + selectedFieldId).addClass('active');
}

$fieldSelect.on('change', showField);
showField();

function pixelToFieldCoords(pixelX, pixelY) {
    const $activeField = getActiveField();
    if ($activeField.length === 0) return null;
    
    const rect = $activeField[0].getBoundingClientRect();
    
    const x = ((pixelX - rect.left) / rect.width) * FIELD_WIDTH_INCHES;
    const y = FIELD_HEIGHT_INCHES - ((pixelY - rect.top) / rect.height) * FIELD_HEIGHT_INCHES;
    
    return { x, y };
}

function fieldToPixelCoords(x, y) {
    const $activeField = getActiveField();
    if ($activeField.length === 0) return null;
    
    const rect = $activeField[0].getBoundingClientRect();
    
    const pixelX = (x / FIELD_WIDTH_INCHES) * rect.width + rect.left;
    const pixelY = rect.height - (y / FIELD_HEIGHT_INCHES) * rect.height + rect.top;
    
    return { x: pixelX, y: pixelY };
}

function createVisualPoint(x, y, className = 'point') {
    const $activeField = getActiveField();
    
    if ($activeField.length === 0) {
        return null;
    }
    
    if ($activeField.attr('id') === 'no_field') {
        return null;
    }
    
    const pixelCoords = fieldToPixelCoords(x, y);
    if (!pixelCoords) return null;
    
    const $point = $('<div></div>')
      .addClass(className)
      .css({
        left: pixelCoords.x + 'px',
        top: pixelCoords.y + 'px',
        position: 'absolute'
      });
    
    $('body').append($point);
    return $point;
}

function createControlPoint(x, y, segmentIndex, controlIndex) {
    const $controlPoint = createVisualPoint(x, y, 'control-point');
    if (!$controlPoint) return null;
    
    $controlPoint.data('segmentIndex', segmentIndex);
    $controlPoint.data('controlIndex', controlIndex);
    $controlPoint.css('pointer-events', 'auto');
    
    $controlPoint.on('mousedown', function(e) {
        e.preventDefault();
        draggingPoint = {
            element: $controlPoint,
            segmentIndex: segmentIndex,
            controlIndex: controlIndex
        };
        $controlPoint.addClass('dragging');
    });
    
    return $controlPoint;
}

$(document).on('mousemove', function(e) {
    if (draggingPoint) {
        const fieldCoords = pixelToFieldCoords(e.pageX, e.pageY);
        if (!fieldCoords) return;
        
        fieldCoords.x = Math.max(0, Math.min(FIELD_WIDTH_INCHES, fieldCoords.x));
        fieldCoords.y = Math.max(0, Math.min(FIELD_HEIGHT_INCHES, fieldCoords.y));
        
        const segment = controlPoints[draggingPoint.segmentIndex];
        if (segment && segment[draggingPoint.controlIndex]) {
            segment[draggingPoint.controlIndex].x = fieldCoords.x;
            segment[draggingPoint.controlIndex].y = fieldCoords.y;
        }
        
        updateAllVisualPoints();
    }
});

$(document).on('mouseup', function() {
    if (draggingPoint) {
        draggingPoint.element.removeClass('dragging');
        draggingPoint = null;
    }
});

function initializeControlPoints(p0, p1) {
    const dx = p1.x - p0.x;
    const dy = p1.y - p0.y;
    
    return [
        new Point(p0.x + dx * 0.2, p0.y + dy * 0.2),
        new Point(p0.x + dx * 0.4, p0.y + dy * 0.4),
        new Point(p0.x + dx * 0.6, p0.y + dy * 0.6),
        new Point(p0.x + dx * 0.8, p0.y + dy * 0.8)
    ];
}

let curvePointsArray = [];

function drawBezierCurve() {
    curvePointsArray = [];
    $('.curve-point, .control-point, .control-line').remove();
    
    if (points.length < 2) {
        return;
    }
    
    const $activeField = getActiveField();
    if ($activeField.length === 0 || $activeField.attr('id') === 'no_field') {
        return;
    }
    
    while (controlPoints.length < points.length - 1) {
        const segmentIndex = controlPoints.length;
        const p0 = points[segmentIndex];
        const p1 = points[segmentIndex + 1];
        controlPoints.push(initializeControlPoints(p0, p1));
    }
    
    controlPoints = controlPoints.slice(0, points.length - 1);
    
    for (let segment = 0; segment < points.length - 1; segment++) {
        const p0 = points[segment];
        const p5 = points[segment + 1];
        const controls = controlPoints[segment];
        
        if (!controls || controls.length !== 4) continue;
        
        const p1 = controls[0];
        const p2 = controls[1];
        const p3 = controls[2];
        const p4 = controls[3];
        
        drawControlLine(p0.x, p0.y, p1.x, p1.y);
        drawControlLine(p1.x, p1.y, p2.x, p2.y);
        drawControlLine(p2.x, p2.y, p3.x, p3.y);
        drawControlLine(p3.x, p3.y, p4.x, p4.y);
        drawControlLine(p4.x, p4.y, p5.x, p5.y);
        
        createControlPoint(p1.x, p1.y, segment, 0);
        createControlPoint(p2.x, p2.y, segment, 1);
        createControlPoint(p3.x, p3.y, segment, 2);
        createControlPoint(p4.x, p4.y, segment, 3);
        
        const numPoints = 100;
        for (let i = 0; i <= numPoints; i++) {
            const t = i / numPoints;
            const curvePoint = quinticBezier(t, p0, p1, p2, p3, p4, p5);
            curvePointsArray.push({
                x: curvePoint.x,
                y: curvePoint.y
            })
            createVisualPoint(curvePoint.x, curvePoint.y, 'curve-point');
        }
    }
    updateSliderRange();
    updateCurveProfile();
}

function drawControlLine(x1, y1, x2, y2) {
    const pixel1 = fieldToPixelCoords(x1, y1);
    const pixel2 = fieldToPixelCoords(x2, y2);
    
    if (!pixel1 || !pixel2) return;
    
    const length = Math.sqrt(Math.pow(pixel2.x - pixel1.x, 2) + Math.pow(pixel2.y - pixel1.y, 2));
    const angle = Math.atan2(pixel2.y - pixel1.y, pixel2.x - pixel1.x) * 180 / Math.PI;
    
    const $line = $('<div></div>')
        .addClass('control-line')
        .css({
            position: 'absolute',
            left: pixel1.x + 'px',
            top: pixel1.y + 'px',
            width: length + 'px',
            height: '2px',
            backgroundColor: 'purple',
            opacity: '0.3',
            transformOrigin: '0 0',
            transform: `rotate(${angle}deg)`,
            pointerEvents: 'none',
            zIndex: 1
        });
    
    $('body').append($line);
}

function updateAllVisualPoints() {
    $('.point, .curve-point, .control-point, .control-line, .user-point').remove();
    
    $('.point-entry').each(function() {
        const x = parseFloat($(this).find('.input-x').val());
        const y = parseFloat($(this).find('.input-y').val());
        
        if (!isNaN(x) && !isNaN(y)) {
            createVisualPoint(x, y, 'user-point');
        }
    });
    
    drawBezierCurve();
}

function createPointEntry(x = '', y = '', heading = '', maxVel = 100, maxAccel = 150) {
    pointCounter++;
    const pointId = `point-${pointCounter}`;
    
    const $pointEntry = $('<div></div>')
      .addClass('point-entry')
      .attr('id', pointId)
      .html(`
        <div class="point-entry-header">
            <span>Point ${pointCounter}</span>
            <button class="delete-point-btn" data-point-id="${pointId}">
                <i class="fa-solid fa-trash"></i> Delete
            </button>
        </div>
        <div class="point-inputs">
            <div class="input-group">
                <label>X (inches)</label>
                <input type="number" class="input-x" value="${x}" step="0.01" placeholder="0.00">
            </div>
            <div class="input-group">
                <label>Y (inches)</label>
                <input type="number" class="input-y" value="${y}" step="0.01" placeholder="0.00">
            </div>
            <div class="input-group">
                <label>Heading (°)</label>
                <input type="number" class="input-heading" value="${heading}" step="1" placeholder="0">
            </div>
        </div>
        <div class="point-inputs" style="margin-top: 10px;">
            <div class="input-group">
                <label>Max Vel (in/s)</label>
                <input type="number" class="input-maxvel" value="${maxVel}" step="5" min="10" max="200">
            </div>
            <div class="input-group">
                <label>Max Accel (in/s²)</label>
                <input type="number" class="input-maxaccel" value="${maxAccel}" step="10" min="10" max="300">
            </div>
        </div>
    `);
    
    $pointsList.append($pointEntry);
    
    $pointEntry.find('.delete-point-btn').on('click', function() {
        const index = $('.point-entry').index($pointEntry);
        $pointEntry.remove();
        
        if (index < controlPoints.length) {
            controlPoints.splice(index, 1);
        }
        if (index > 0 && index - 1 < controlPoints.length) {
            controlPoints.splice(index - 1, 1);
        }
        
        updatePointsArray();
        updateAllVisualPoints();
    });
    
    $pointEntry.find('input').on('input', function() {
        updatePointsArray();
        updateAllVisualPoints();
    });
    
    updatePointsArray();
    updateAllVisualPoints();
}

function updatePointsArray() {
    points = [];
    $('.point-entry').each(function() {
        const x = $(this).find('.input-x').val();
        const y = $(this).find('.input-y').val();
        const heading = $(this).find('.input-heading').val();
        const maxVel = $(this).find('.input-maxvel').val();
        const maxAccel = $(this).find('.input-maxaccel').val();
        
        points.push({
            x: parseFloat(x) || 0,
            y: parseFloat(y) || 0,
            heading: parseFloat(heading) || 0,
            maxVelocity: parseFloat(maxVel) || 100,
            maxAcceleration: parseFloat(maxAccel) || 150
        });
    });
}

$addPointButton.on('click', function() {
    createPointEntry();
});

$removePointsButton.on('click', function() {
    points = [];
    controlPoints = [];
    pointCounter = 0;
    $pointsList.empty();
    $('.point, .curve-point, .control-point, .control-line, .user-point').remove();
    $('#graph_container').hide();
});

$fieldSelect.on('change', function() {
    showField();
    updateAllVisualPoints();
});

let currentFrame = 0;
const robot = $('#robot');

function getRobotDimensions() {
    const width = parseFloat($('.robot_dimensions_input').eq(0).val()) || 18;
    const height = parseFloat($('.robot_dimensions_input').eq(1).val()) || 18;
    
    return { width, height };
}

function calculateHeading(fromPoint, toPoint) {
    const dx = toPoint.x - fromPoint.x;
    const dy = toPoint.y - fromPoint.y;
    
    const angleRadians = Math.atan2(dy, dx);
    const angleDegrees = angleRadians * (180 / Math.PI);
    
    return angleDegrees;
}

let curveHeadingAngles = [];

function curveHeadingCalculation() {
    curveHeadingAngles = [];
    for(let i = 0; i < curvePointsArray.length - 1; i++) {
        let fromPoint = curvePointsArray[i];
        let toPoint = curvePointsArray[i+1];
        let angle = calculateHeading(fromPoint, toPoint);
        curveHeadingAngles.push(angle);
    }
}

function setRobotSize() {
    const { width, height } = getRobotDimensions();
    const scaledWidth = width * 3;
    const scaledHeight = height * 3;

    robot.css({
        width: scaledWidth + 'px',
        height: scaledHeight + 'px'
    });
}

function updateRobot(frame) {
    if (curvePointsArray.length === 0) return;
    
    const safeFrame = Math.min(frame, curvePointsArray.length - 1);
    const fieldCoords = curvePointsArray[safeFrame];
    const pixelCoords = fieldToPixelCoords(fieldCoords.x, fieldCoords.y);
    
    if (!pixelCoords) return;
    
    const heading = curveHeadingAngles[safeFrame] || 0;
    
    robot.css({
        left: pixelCoords.x + 'px',
        top: pixelCoords.y + 'px',
        transform: `translate(-50%, -50%) rotate(${heading}deg)`
    });
}

function updateSliderRange() {
    const maxFrame = Math.max(0, curvePointsArray.length - 1);
    $('#keyFrameSlider').attr('max', maxFrame);
    
    currentFrame = 0;
    $('#keyFrameSlider').val(0);
    $('#sliderValue').text('0 / ' + maxFrame);
    
    curveHeadingCalculation();
    setRobotSize();
    updateRobot(0);
}

$('#keyFrameSlider').on('input', function() {
    currentFrame = parseInt($(this).val());
    const maxFrame = parseInt($(this).attr('max'));
    $('#sliderValue').text(currentFrame + ' / ' + maxFrame);
    updateRobot(currentFrame);
});

$('.robot_dimensions_input').on('input', function() {
    setRobotSize();
});

// Function to create a single graph
function createGraph(containerId, data, dataKey, title, yLabel, color = '#2196F3') {
    const container = document.getElementById(containerId);
    if (!container) return;
    
    const { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } = window.Recharts;
    
    const element = React.createElement(ResponsiveContainer, { width: '100%', height: 250 },
        React.createElement(LineChart, { 
            data: data, 
            margin: { top: 5, right: 20, bottom: 5, left: 20 }
        },
            React.createElement(CartesianGrid, { strokeDasharray: '3 3' }),
            React.createElement(XAxis, { 
                dataKey: 'frame',
                label: { value: 'Frame', position: 'insideBottom', offset: -5 }
            }),
            React.createElement(YAxis, { 
                label: { value: yLabel, angle: -90, position: 'insideLeft' }
            }),
            React.createElement(Tooltip),
            React.createElement(Legend),
            React.createElement(Line, { 
                type: 'monotone', 
                dataKey: dataKey, 
                stroke: color, 
                strokeWidth: 2,
                dot: false,
                name: title
            })
        )
    );
    
    const root = ReactDOM.createRoot(container);
    root.render(element);
}

// Function to create XY scatter plot
function createXYPlot(containerId, data) {
    const container = document.getElementById(containerId);
    if (!container) return;
    
    const { ScatterChart, Scatter, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } = window.Recharts;
    
    const element = React.createElement(ResponsiveContainer, { width: '100%', height: 300 },
        React.createElement(ScatterChart, { 
            margin: { top: 10, right: 20, bottom: 20, left: 20 }
        },
            React.createElement(CartesianGrid, { strokeDasharray: '3 3' }),
            React.createElement(XAxis, { 
                type: 'number', 
                dataKey: 'x', 
                name: 'X', 
                unit: ' in',
                domain: [0, 144]
            }),
            React.createElement(YAxis, { 
                type: 'number', 
                dataKey: 'y', 
                name: 'Y', 
                unit: ' in',
                domain: [0, 144]
            }),
            React.createElement(Tooltip, { cursor: { strokeDasharray: '3 3' } }),
            React.createElement(Scatter, { 
                name: 'Path', 
                data: data, 
                fill: '#2196F3',
                line: { stroke: '#2196F3', strokeWidth: 2 }
            })
        )
    );
    
    const root = ReactDOM.createRoot(container);
    root.render(element);
}

function updateCurveProfile() {
    const $graphContainer = $('#graph_container');
    
    if (points.length < 2 || curvePointsArray.length === 0) {
        $graphContainer.hide();
        return;
    }
    
    $graphContainer.show().empty();
    
    const curveProfile = generateQuinticPathProfile();
    
    if (curveProfile.length === 0) {
        $graphContainer.hide();
        return;
    }
    
    // Create HTML structure for all graphs
    const graphsHTML = `
        <div class="graph-card">
            <div class="graph-title">Path Trajectory (X vs Y)</div>
            <div id="graph-xy"></div>
        </div>
        <div class="graph-card">
            <div class="graph-title">Linear Velocity</div>
            <div id="graph-velocity"></div>
        </div>
        <div class="graph-card">
            <div class="graph-title">Angular Velocity</div>
            <div id="graph-angular"></div>
        </div>
        <div class="graph-card">
            <div class="graph-title">Path Curvature</div>
            <div id="graph-curvature"></div>
        </div>
        <div class="graph-card">
            <div class="graph-title">Acceleration</div>
            <div id="graph-acceleration"></div>
        </div>
        <div class="graph-card">
            <div class="graph-title">Jerk</div>
            <div id="graph-jerk"></div>
        </div>
        <div class="graph-card">
            <div class="graph-title">Robot Heading</div>
            <div id="graph-heading"></div>
        </div>
    `;
    
    $graphContainer.html(graphsHTML);
    
    // Wait for DOM to be ready, then create graphs
    setTimeout(() => {
        createXYPlot('graph-xy', curveProfile);
        createGraph('graph-velocity', curveProfile, 'linearVelocity', 'Velocity', 'Velocity (in/s)');
        createGraph('graph-angular', curveProfile, 'angularVelocity', 'Angular Velocity', 'ω (rad/s)');
        createGraph('graph-curvature', curveProfile, 'curvature', 'Curvature', 'κ');
        createGraph('graph-acceleration', curveProfile, 'acceleration', 'Acceleration', 'Accel (in/s²)');
        createGraph('graph-jerk', curveProfile, 'jerk', 'Jerk', 'Jerk (in/s³)');
        createGraph('graph-heading', curveProfile, 'heading', 'Heading', 'Heading (°)');
    }, 100);
}

}); // End of $(document).ready