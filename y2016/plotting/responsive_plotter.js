"use strict";

// Create anonymous namespace to avoid leaking these functions.
(function() {
// Convenience function that fetches a file and extracts the array buffer.
function fetch_ArrayBuffer(url) { // returns Promise<ArrayBuffer>
	return fetch(url).then(function(val) { return val.arrayBuffer(); });
}

// Two dimensional vector.
class Point2d {
	constructor(x, y) {
		this.x = x;
		this.y = y;
	}
	sub(o) { return new Point2d(this.x - o.x, this.y - o.y); }
	add(o) { return new Point2d(this.x + o.x, this.y + o.y); }
};

// Drag event object for updating a Point2d class by reference.
class PointDrag {
	constructor(point) { // point : Point2d
		this.point = point;
	}
	doDrag(delta) {
		this.point.x += delta.x;
		this.point.y += delta.y;
	}
	doMouseUp(e) { return null; }
}

// Takes in a canvas and adds the ability to click and drag the offset point.
// scaleX and scaleY are allowed to be locked or not-locked as the scroll events
// must be handled by the user of this class.
//
// offset.x and offset.y are defined in reference to the model-scale, not the viewport.
// Thus to move the viewport you need to use offset.x * scaleX, etc.
//
// scaleX and scaleY aren't automatically applied to the rendering context because
// differing x and y scales mess up the line width rendering.
class ClickDrag {
	constructor(canvas) {
 		canvas.addEventListener('selectstart', function(e) { e.preventDefault(); return false; }, false);
  	canvas.addEventListener('mousemove', this.handleMouseMove.bind(this), true);
		canvas.addEventListener('mousedown', this.handleMouseDown.bind(this), true);
		canvas.addEventListener('mouseup', this.handleMouseUp.bind(this), true);
		this.canvas = canvas;
		this.dragger = null;
		this.offset = new Point2d(0, 0);
		this.scaleX = 1.0;
		this.scaleY = 1.0;
		this.old_point = null;
		this.on_redraw = null;
	}
	translateContext(ctx) { // ctx : CanvasRenderingContext2D 
		ctx.translate(this.offset.x * this.scaleX, this.offset.y * this.scaleY);
	}
	handleMouseMove(e) { // e : MouseEvent
		let pos = new Point2d(e.offsetX / this.scaleX, e.offsetY / this.scaleY);
		if (this.old_point !== null && this.dragger !== null) {
			this.dragger.doDrag(pos.sub(this.old_point));
			if (this.on_redraw !== null) { this.on_redraw(); }
		}
		this.old_point = pos;
  }
	get width() { return this.canvas.width; }
	handleMouseDown(e) { // e : MouseEvent
		this.dragger = new PointDrag(this.offset);
	}
	handleMouseUp(e) { // e : MouseEvent
		if (this.dragger === null) { return; }
		if (this.dragger.doMouseUp !== undefined) {
			this.dragger = this.dragger.doMouseUp(e);
			if (this.on_redraw !== null) { this.on_redraw(); }
		}
	}
	multiplyScale(evt, dx, dy) {
		let x_cur = evt.offsetX / this.scaleX - this.offset.x;
		let y_cur = evt.offsetY / this.scaleY - this.offset.y;
		this.scaleX *= dx;
		this.scaleY *= dy;
		this.offset.x = evt.offsetX / this.scaleX - x_cur;
		this.offset.y = evt.offsetY / this.scaleY - y_cur;
		if (this.on_redraw !== null) { this.on_redraw(); }
	}
};

// This represents a particular downsampling of a timeseries dataset.
// For the base level, data = min = max.
// This is required
class MipMapLevel {
	// All arrays should be the same size.
	constructor(data, min, max, scale) { // data : Float32Array, min : Float32Array, max : Float32Array, scale : Integer
		this.data = data;
		this.min = min;
		this.max = max;
		this.scale = scale;
	}
	get length() { return this.data.length; }
	// Uses box filter to downsample. Guassian may be an improvement.
	downsample() {
		let new_length = (this.data.length / 2) | 0;
		let data = new Float32Array(new_length);
		let min = new Float32Array(new_length);
		let max = new Float32Array(new_length);
		for (let i = 0; i < new_length; i++) {
			let i1 = i * 2;
			let i2 = i1 + 1
			data[i] = (this.data[i1] + this.data[i2]) / 2.0;
			min[i] = Math.min(this.min[i1], this.min[i2]);
			max[i] = Math.max(this.max[i1], this.max[i2]);
		}
		return new MipMapLevel(data, min, max, this.scale * 2);
	}
	static makeBaseLevel(data) { // data : Float32Array
		return new MipMapLevel(data, data, data, 1);
	}
	clipv(v) { // v : Numeric
		return Math.min(Math.max(v|0, 0), this.length - 1);
	}
	draw(ctx, clickDrag) { // ctx : CanvasRenderingContext2D, clickDrag : ClickDrag
		let scalex = clickDrag.scaleX * this.scale;
		let scaley = clickDrag.scaleY;
		let stx = -clickDrag.offset.x / this.scale;
		let edx = stx + clickDrag.width / scalex;
		let sti = this.clipv(stx - 2);
		let edi = this.clipv(edx + 2);
		ctx.beginPath();
		ctx.fillStyle="#00ff00";
		ctx.moveTo(sti * scalex, scaley * this.min[sti]);
		for (let i = sti + 1; i <= edi; i++) {
			ctx.lineTo(i * scalex, scaley * this.min[i]);
		}
		for (let i = edi; i >= sti; --i) {
			ctx.lineTo(i * scalex, scaley * this.max[i]);
		}
		ctx.fill();
		ctx.closePath();

		ctx.beginPath();
		ctx.strokeStyle="#008f00";
		ctx.moveTo(sti * scalex, scaley * this.data[sti]);
		for (let i = sti + 1; i <= edi; i++) {
			ctx.lineTo(i * scalex, scaley * this.data[i]);
		}
		ctx.stroke();
		ctx.closePath();
	}
};

// This class represents all downsampled data levels.
// This must only be used for descrite time-series data.
class MipMapper {
	constructor(samples) { // samples : Float32Array
		this.levels = [];
		let level = MipMapLevel.makeBaseLevel(samples);
		this.levels.push(level);
		while (level.length > 2) {
			level = level.downsample();
			this.levels.push(level);
		}
	}
	// Find the level such that samples are spaced at most 2 pixels apart
	getLevel(scale) { // scale : Float
		let level = this.levels[0];
		for (let i = 0; i < this.levels.length; ++i) {
			// Someone who understands nyquist could probably improve this.
			if (this.levels[i].scale * scale <= 4.0) {
				level = this.levels[i];
			}
		}
		return level;
	}
}

// Custom HTML element for x-responsive-plotter.
// This should be the main entry-point for this code.
class ResponsivePlotter extends HTMLElement {
	createdCallback() {
		this.data = null;
		this.canvas = document.createElement("canvas");
		this.canvas.style["background"] = "inherit";
		this.canvas.addEventListener("mousewheel", this.doScroll.bind(this), false);
		this.clickDrag = new ClickDrag(this.canvas);
		this.clickDrag.on_redraw = this.doRedraw.bind(this);
		this.clickDrag.offset.y = 160; //320;
		this.updateInnerData();
		this.appendChild(this.canvas);
	}
	doScroll(e) {
		let factor = (e.wheelDelta < 0) ? 0.9 : 1.1;
		this.clickDrag.multiplyScale(e, e.shiftKey ? 1.0 : factor, e.shiftKey ? factor : 1.0);
		e.preventDefault();
	}
	doRedraw() {
		this.redraw();
	}
	updateInnerData() {
		this.canvas.width = this.getAttribute("width") || 640;
		this.canvas.height = this.getAttribute("height") || 480;
		fetch_ArrayBuffer(this.getAttribute("data")).then(this.dataUpdated.bind(this));
		this.redraw();
	}
	attributeChangedCallback(attrName, oldVal, newVal) {
		if (attrName == "width") {
			this.canvas.width = this.getAttribute("width") || 640;
			this.redraw();
		} else if (attrName == "height") {
			this.canvas.width = this.getAttribute("height") || 480;
			this.redraw();
		} else if (attrName == "data") {
			// Should this clear out the canvas if a different url is selected?
			// Well it does.
			this.data = null;
			fetch_ArrayBuffer(this.getAttribute("data")).then(this.dataUpdated.bind(this));
		}
	}
	dataUpdated(data) { // data : ArrayBuffer
		this.data = new MipMapper(new Float32Array(data));
		this.redraw();
	}
	redraw() {
		let ctx = this.canvas.getContext('2d');
		ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
		ctx.save();
		this.clickDrag.translateContext(ctx);
		if (this.data !== null) {
			let level = this.data.getLevel(this.clickDrag.scaleX);
			level.draw(ctx, this.clickDrag);
		}
		ctx.restore();
	}
};


// TODO(parker): This is chrome-specific v0 version of this switch to v1 when available.
document.registerElement('x-responsive-plotter', ResponsivePlotter);
})();
