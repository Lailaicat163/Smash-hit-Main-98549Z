$(document).ready(function() {
const $fieldSelect = $('#field_select');
const $fields = $('.field');
const $removePointsButton = $('#remove_button');
const $addPointButton = $('#add_button');
const $pointsList = $('#points_list');
let points = []; // Array to store all points
let pointCounter = 0; // Counter for point numbers
let controlPoints = []; // Array to store control points for each segment
let draggingPoint = null; // Currently dragging point

class Point {
  constructor(x, y) {
    this.x = x;
    this.y = y;
  }
}

//Make points on quintic bezier curve
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

// First derivative (velocity)
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

// Second derivative (acceleration)
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

// Third derivative (jerk)
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
    // Position
    const position = quinticBezier(t, p0, p1, p2, p3, p4, p5);
    
    // First derivative (velocity vector)
    const velocity = quinticBezierDerivative(t, p0, p1, p2, p3, p4, p5);
    
    // Second derivative (acceleration vector)
    const acceleration = quinticBezierSecondDerivative(t, p0, p1, p2, p3, p4, p5);
    
    // Third derivative (jerk vector)
    const jerk = quinticBezierThirdDerivative(t, p0, p1, p2, p3, p4, p5);
    
    // Linear velocity (magnitude of velocity vector)
    const linearVelocity = Math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    
    // Heading (angle of velocity vector)
    const heading = Math.atan2(velocity.y, velocity.x);
    
    // Curvature: κ = (x' * y'' - y' * x'') / (x'² + y'²)^(3/2)
    const curvature = (velocity.x * acceleration.y - velocity.y * acceleration.x) / 
                      Math.pow(velocity.x * velocity.x + velocity.y * velocity.y, 1.5);
    
    // Angular velocity: ω = v * κ
    const angularVelocity = linearVelocity * curvature;
    
    // Linear acceleration (magnitude of acceleration vector)
    const linearAcceleration = Math.sqrt(acceleration.x * acceleration.x + 
                                         acceleration.y * acceleration.y);
    
    // Tangential acceleration (along path direction)
    const tangentialAcceleration = (velocity.x * acceleration.x + 
                                    velocity.y * acceleration.y) / linearVelocity;
    
    // Centripetal acceleration (perpendicular to path)
    const centripetalAcceleration = linearVelocity * linearVelocity * Math.abs(curvature);
    
    // Linear jerk (rate of change of acceleration)
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
        velocity: velocity,        // Vector form
        acceleration: acceleration, // Vector form
        jerk: jerk                 // Vector form
    };
}

function generateQuinticPathProfile() {
    const pathProfile = [];
    
    if (points.length < 2) return pathProfile;
    
    // For each segment in your curve
    for (let segment = 0; segment < points.length - 1; segment++) {
        const p0 = points[segment];
        const p5 = points[segment + 1];
        const controls = controlPoints[segment];
        
        if (!controls || controls.length !== 4) continue;
        
        const p1 = controls[0];
        const p2 = controls[1];
        const p3 = controls[2];
        const p4 = controls[3];
        
        // Use the target point's max velocity and acceleration for this segment
        const segmentMaxVel = p5.maxVelocity || 100;
        const segmentMaxAccel = p5.maxAcceleration || 150;
        
        const numPoints = 100;
        const dt = 1.0 / numPoints;
        
        for (let i = 0; i <= numPoints; i++) {
            const t = i / numPoints;
            
            const props = calculatePathProperties(t, p0, p1, p2, p3, p4, p5, dt);
            
            // Skip if velocity is invalid
            if (!props.linearVelocity || isNaN(props.linearVelocity)) continue;
            
            // Adjust velocity based on curvature
            const curvatureVelocity = segmentMaxVel / (1 + Math.abs(props.curvature) * 30);
            const targetVelocity = Math.min(segmentMaxVel, curvatureVelocity);
            
            // Scale velocities and accelerations
            const velocityScale = targetVelocity / (props.linearVelocity || 1);
            
            pathProfile.push({
                x: props.position.x,
                y: props.position.y,
                heading: props.heading,
                linearVelocity: targetVelocity,
                angularVelocity: props.angularVelocity * velocityScale,
                curvature: props.curvature,
                acceleration: Math.min(segmentMaxAccel, Math.abs(props.tangentialAcceleration * velocityScale)),
                centripetalAccel: props.centripetalAcceleration,
                jerk: props.linearJerk,
                segment: segment
            });
        }
    }
    
    return pathProfile;
}

// Constants for field dimensions
const FIELD_WIDTH_INCHES = 144;
const FIELD_HEIGHT_INCHES = 144;

// Function to get the active field
function getActiveField() {
    const $active = $('.field.active');
    return $active;
}

// Function to show the selected field
function showField() {
    $fields.removeClass('active');
    const selectedFieldId = $fieldSelect.val();
    $('#' + selectedFieldId).addClass('active');
}

// Initialize field display
$fieldSelect.on('change', showField);
showField();

// Convert pixel coordinates to field inches
function pixelToFieldCoords(pixelX, pixelY) {
    const $activeField = getActiveField();
    if ($activeField.length === 0) return null;
    
    const rect = $activeField[0].getBoundingClientRect();
    
    const x = ((pixelX - rect.left) / rect.width) * FIELD_WIDTH_INCHES;
    const y = FIELD_HEIGHT_INCHES - ((pixelY - rect.top) / rect.height) * FIELD_HEIGHT_INCHES;
    
    return { x, y };
}

// Convert field inches to pixel coordinates
function fieldToPixelCoords(x, y) {
    const $activeField = getActiveField();
    if ($activeField.length === 0) return null;
    
    const rect = $activeField[0].getBoundingClientRect();
    
    const pixelX = (x / FIELD_WIDTH_INCHES) * rect.width + rect.left;
    const pixelY = rect.height - (y / FIELD_HEIGHT_INCHES) * rect.height + rect.top;
    
    return { x: pixelX, y: pixelY };
}

// Function to create visual point on the field
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
    
    // Create visual point
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

// Function to create draggable control point
function createControlPoint(x, y, segmentIndex, controlIndex) {
    const $controlPoint = createVisualPoint(x, y, 'control-point');
    if (!$controlPoint) return null;
    
    $controlPoint.data('segmentIndex', segmentIndex);
    $controlPoint.data('controlIndex', controlIndex);
    $controlPoint.css('pointer-events', 'auto');
    
    // Make it draggable
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

// Global mouse handlers for dragging
$(document).on('mousemove', function(e) {
    if (draggingPoint) {
        const fieldCoords = pixelToFieldCoords(e.pageX, e.pageY);
        if (!fieldCoords) return;
        
        // Clamp to field boundaries
        fieldCoords.x = Math.max(0, Math.min(FIELD_WIDTH_INCHES, fieldCoords.x));
        fieldCoords.y = Math.max(0, Math.min(FIELD_HEIGHT_INCHES, fieldCoords.y));
        
        // Update control point position
        const segment = controlPoints[draggingPoint.segmentIndex];
        if (segment && segment[draggingPoint.controlIndex]) {
            segment[draggingPoint.controlIndex].x = fieldCoords.x;
            segment[draggingPoint.controlIndex].y = fieldCoords.y;
        }
        
        // Redraw everything
        updateAllVisualPoints();
    }
});

$(document).on('mouseup', function() {
    if (draggingPoint) {
        draggingPoint.element.removeClass('dragging');
        draggingPoint = null;
    }
});

// Initialize control points for a segment
function initializeControlPoints(p0, p1) {
    const dx = p1.x - p0.x;
    const dy = p1.y - p0.y;
    
    // Create 4 control points evenly spaced between start and end
    return [
        new Point(p0.x + dx * 0.2, p0.y + dy * 0.2),
        new Point(p0.x + dx * 0.4, p0.y + dy * 0.4),
        new Point(p0.x + dx * 0.6, p0.y + dy * 0.6),
        new Point(p0.x + dx * 0.8, p0.y + dy * 0.8)
    ];
}

let curvePointsArray = [];

// Function to draw Bézier curve with control points
function drawBezierCurve() {
    curvePointsArray = [];
    // Remove existing curve points
    $('.curve-point, .control-point, .control-line').remove();
    
    if (points.length < 2) {
        return;
    }
    
    const $activeField = getActiveField();
    if ($activeField.length === 0 || $activeField.attr('id') === 'no_field') {
        return;
    }
    
    // Ensure we have control points for each segment
    while (controlPoints.length < points.length - 1) {
        const segmentIndex = controlPoints.length;
        const p0 = points[segmentIndex];
        const p1 = points[segmentIndex + 1];
        controlPoints.push(initializeControlPoints(p0, p1));
    }
    
    // Remove extra control points if we have fewer segments now
    controlPoints = controlPoints.slice(0, points.length - 1);
    
    // Draw each segment
    for (let segment = 0; segment < points.length - 1; segment++) {
        const p0 = points[segment];
        const p5 = points[segment + 1];
        const controls = controlPoints[segment];
        
        if (!controls || controls.length !== 4) continue;
        
        const p1 = controls[0];
        const p2 = controls[1];
        const p3 = controls[2];
        const p4 = controls[3];
        
        // Draw control lines
        drawControlLine(p0.x, p0.y, p1.x, p1.y);
        drawControlLine(p1.x, p1.y, p2.x, p2.y);
        drawControlLine(p2.x, p2.y, p3.x, p3.y);
        drawControlLine(p3.x, p3.y, p4.x, p4.y);
        drawControlLine(p4.x, p4.y, p5.x, p5.y);
        
        // Draw control points
        createControlPoint(p1.x, p1.y, segment, 0);
        createControlPoint(p2.x, p2.y, segment, 1);
        createControlPoint(p3.x, p3.y, segment, 2);
        createControlPoint(p4.x, p4.y, segment, 3);
        
        // Draw quintic Bézier curve
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

// Draw a line between control points
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

// Function to update all visual points
function updateAllVisualPoints() {
    // Remove all existing points
    $('.point, .curve-point, .control-point, .control-line, .user-point').remove();
    
    // Create visual points for all entries
    $('.point-entry').each(function() {
        const x = parseFloat($(this).find('.input-x').val());
        const y = parseFloat($(this).find('.input-y').val());
        
        if (!isNaN(x) && !isNaN(y)) {
            createVisualPoint(x, y, 'user-point');
        }
    });
    
    // Draw curve with control points
    drawBezierCurve();
}

// Function to create a new point entry
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
        
        // Remove associated control points
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

// Function to update points array
// Function to update points array
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

// Add button click handler
$addPointButton.on('click', function() {
    createPointEntry();
});

// Remove all points
// Update the existing remove button click handler
$removePointsButton.on('click', function() {
    points = [];
    controlPoints = [];
    pointCounter = 0;
    $pointsList.empty();
    $('.point, .curve-point, .control-point, .control-line, .user-point').remove();
    $('#curve_list').hide(); // ADD THIS
    $('#curve_controls').hide(); // ADD THIS
});

// Update visual points when field changes
$fieldSelect.on('change', function() {
    showField();
    updateAllVisualPoints();
});

//Start robot animation

//Gets user input on robot dimensions
function getRobotDimensions() {
    const width = parseFloat($('.robot_dimensions_input').eq(0).val()) || 18;
    const height = parseFloat($('.robot_dimensions_input').eq(1).val()) || 18;
    
    return { width, height };
}

//Start robot animation
let currentFrame = 0;
const robot = $('#robot');

//Gets user input on robot dimensions
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

//Calculates heading at each curve point
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

// Update slider range when curve changes
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

// Slider input handler
$('#keyFrameSlider').on('input', function() {
    currentFrame = parseInt($(this).val());
    const maxFrame = parseInt($(this).attr('max'));
    $('#sliderValue').text(currentFrame + ' / ' + maxFrame);
    updateRobot(currentFrame);
});

// Update robot size when dimensions change
$('.robot_dimensions_input').on('input', function() {
    setRobotSize();
});

// Curve profile variable
let curveProfile = [];

// Generate and display curve profile
function updateCurveProfile() {
    const $curveList = $('#curve_list');
    
    // Hide if less than 2 points
    if (points.length < 2 || curvePointsArray.length === 0) {
        $curveList.hide();
        return;
    }
    
    // Show list
    $curveList.show().empty();
    
    // Generate profile
    curveProfile = generateQuinticPathProfile();
    
    if (curveProfile.length === 0) {
        $curveList.hide();
        return;
    }
    
    // Group by segment
    const segmentGroups = {};
    curveProfile.forEach(point => {
        if (!segmentGroups[point.segment]) {
            segmentGroups[point.segment] = [];
        }
        segmentGroups[point.segment].push(point);
    });
    
    // Display each segment
    Object.keys(segmentGroups).forEach(segmentIndex => {
        const segmentPoints = segmentGroups[segmentIndex];
        const displayInterval = Math.max(1, Math.floor(segmentPoints.length / 20));
        
        // Segment header
        const $segmentHeader = $('<div></div>')
            .addClass('curve-entry')
            .css({
                'background-color': '#1a3a1a',
                'border-color': '#4CAF50'
            })
            .html(`
                <div class="curve-entry-header">
                    Segment ${parseInt(segmentIndex) + 1} 
                    (Point ${parseInt(segmentIndex) + 1} → ${parseInt(segmentIndex) + 2})
                </div>
                <div class="curve-data">
                    <div class="curve-data-item">
                        <span class="curve-data-label">Points:</span>
                        <span class="curve-data-value">${segmentPoints.length}</span>
                    </div>
                    <div class="curve-data-item">
                        <span class="curve-data-label">Max Vel:</span>
                        <span class="curve-data-value">${points[parseInt(segmentIndex) + 1].maxVelocity} in/s</span>
                    </div>
                </div>
            `);
        
        $curveList.append($segmentHeader);
        
        // Display sample points from segment
        for (let i = 0; i < segmentPoints.length; i += displayInterval) {
            const point = segmentPoints[i];
            
            const $curveEntry = $('<div></div>')
                .addClass('curve-entry')
                .html(`
                    <div class="curve-entry-header">Frame ${curveProfile.indexOf(point)}</div>
                    <div class="curve-data">
                        <div class="curve-data-item">
                            <span class="curve-data-label">X:</span>
                            <span class="curve-data-value">${point.x.toFixed(2)}"</span>
                        </div>
                        <div class="curve-data-item">
                            <span class="curve-data-label">Y:</span>
                            <span class="curve-data-value">${point.y.toFixed(2)}"</span>
                        </div>
                        <div class="curve-data-item">
                            <span class="curve-data-label">Heading:</span>
                            <span class="curve-data-value">${(point.heading * 180 / Math.PI).toFixed(1)}°</span>
                        </div>
                        <div class="curve-data-item">
                            <span class="curve-data-label">Velocity:</span>
                            <span class="curve-data-value">${point.linearVelocity.toFixed(1)} in/s</span>
                        </div>
                        <div class="curve-data-item">
                            <span class="curve-data-label">ω:</span>
                            <span class="curve-data-value">${point.angularVelocity.toFixed(3)} rad/s</span>
                        </div>
                        <div class="curve-data-item">
                            <span class="curve-data-label">κ:</span>
                            <span class="curve-data-value">${point.curvature.toFixed(4)}</span>
                        </div>
                        <div class="curve-data-item">
                            <span class="curve-data-label">Accel:</span>
                            <span class="curve-data-value">${point.acceleration.toFixed(1)} in/s²</span>
                        </div>
                        <div class="curve-data-item">
                            <span class="curve-data-label">Centrip:</span>
                            <span class="curve-data-value">${point.centripetalAccel.toFixed(1)} in/s²</span>
                        </div>
                    </div>
                `);
            
            $curveList.append($curveEntry);
        }
    });
}

}); // End of $(document).ready