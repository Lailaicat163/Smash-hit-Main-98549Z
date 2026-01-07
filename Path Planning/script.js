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
function createPointEntry(x = '', y = '', heading = '') {
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
function updatePointsArray() {
    points = [];
    $('.point-entry').each(function() {
        const x = $(this).find('.input-x').val();
        const y = $(this).find('.input-y').val();
        const heading = $(this).find('.input-heading').val();
        
        points.push({
            x: parseFloat(x) || 0,
            y: parseFloat(y) || 0,
            heading: parseFloat(heading) || 0
        });
    });
}

// Add button click handler
$addPointButton.on('click', function() {
    createPointEntry();
});

// Remove all points
$removePointsButton.on('click', function() {
    points = [];
    controlPoints = [];
    pointCounter = 0;
    $pointsList.empty();
    $('.point, .curve-point, .control-point, .control-line, .user-point').remove();
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

let currentFrame = 0;

function calculateHeading(fromPoint, toPoint) {
    const dx = toPoint.x - fromPoint.x;
    const dy = toPoint.y - fromPoint.y;
    
    // atan2 returns radians, convert to degrees
    const angleRadians = Math.atan2(dy, dx);
    const angleDegrees = angleRadians * (180 / Math.PI);
    
    return angleDegrees;
}

curveHeadingAngles = [];
//Calculates heading at each curve point
function curveHeadingCalculation() {
    for(let i = 0; i < curvePointsArray.length; i+=2) {
        let fromPoint = curvePointsArray.at[i];
        let toPoint = curvePointsArray.at[i+1];
        calculateHeading(fromPoint, toPoint);
        curveHeadingAngles.push(angleDegrees);
    }
}

function animate() {
    // Update position
    currentFrame++;
    
    // Draw/move robot
    updateRobot();
    
    // Keep animating
    requestAnimationFrame(animate);
}

// Start animation
animate();

});