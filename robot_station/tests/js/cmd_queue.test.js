"use strict";

const assert = require("assert");
const path = require("path");

const { create, shouldDrop } = require(
  path.join(__dirname, "..", "..", "robot_station", "web", "static", "js", "cmdqueue.js")
);

function test(name, fn) {
  try {
    fn();
    console.log("ok - " + name);
  } catch (err) {
    console.error("not ok - " + name);
    console.error(err && err.stack ? err.stack : err);
    process.exitCode = 1;
  }
}

test("shouldDrop: stream only", () => {
  assert.strictEqual(shouldDrop({ t: "arm", stream: true }), true);
  assert.strictEqual(shouldDrop({ t: "arm" }), false);
  assert.strictEqual(shouldDrop({ t: "cart" }), true);
  assert.strictEqual(shouldDrop({ t: "teach" }), true);
  assert.strictEqual(shouldDrop({ t: "home" }), false);
});

test("non-stream arm is queued and sent", () => {
  let t = 0;
  const q = create({ ttlMs: 1000, now: () => t });
  const sent = [];
  assert.strictEqual(q.enqueue({ t: "arm", q: [0, 0, 0, 0, 0, 0] }, () => {}), true);
  assert.strictEqual(q.size(), 1);
  assert.strictEqual(q.flush((obj) => { sent.push(obj.t); return true; }, () => {}), true);
  assert.deepStrictEqual(sent, ["arm"]);
  assert.strictEqual(q.size(), 0);
});

test("estop clears the queue and is sent first", () => {
  let t = 0;
  const q = create({ ttlMs: 1000, now: () => t });
  const dropped = [];
  const sent = [];
  q.enqueue({ t: "home" }, (obj, reason) => dropped.push([obj.t, reason]));
  q.enqueue({ t: "e" }, (obj, reason) => dropped.push([obj.t, reason]));
  assert.deepStrictEqual(dropped.map((row) => row[0]), ["home"]);
  assert.strictEqual(q.hasEstop(), true);
  q.flush((obj) => { sent.push(obj.t); return true; }, () => {});
  assert.deepStrictEqual(sent, ["e"]);
});

test("duplicate estop is ignored", () => {
  const q = create({ ttlMs: 1000, now: () => 0 });
  assert.strictEqual(q.enqueue({ t: "e" }, () => {}), true);
  assert.strictEqual(q.enqueue({ t: "e" }, () => {}), false);
  assert.strictEqual(q.size(), 1);
});

test("ttl cancels an old command", () => {
  let t = 0;
  const q = create({ ttlMs: 1000, now: () => t });
  const dropped = [];
  const sent = [];
  q.enqueue({ t: "home" }, (obj, reason) => dropped.push([obj.t, reason]));
  t = 1001;
  q.flush((obj) => { sent.push(obj.t); return true; }, (obj, reason) => dropped.push([obj.t, reason]));
  assert.deepStrictEqual(sent, []);
  assert.strictEqual(dropped.length, 1);
  assert.strictEqual(dropped[0][0], "home");
  assert.strictEqual(q.size(), 0);
});

test("estop never expires", () => {
  let t = 0;
  const q = create({ ttlMs: 1000, now: () => t });
  q.enqueue({ t: "e" }, () => {});
  t = 999999;
  const sent = [];
  q.flush((obj) => { sent.push(obj.t); return true; }, () => {});
  assert.deepStrictEqual(sent, ["e"]);
});

test("overflow drops the oldest non-estop", () => {
  const q = create({ ttlMs: 1000, maxItems: 2, now: () => 0 });
  const dropped = [];
  q.enqueue({ t: "home" }, (obj, reason) => dropped.push(obj.t));
  q.enqueue({ t: "power" }, (obj, reason) => dropped.push(obj.t));
  q.enqueue({ t: "link" }, (obj, reason) => dropped.push(obj.t));
  assert.deepStrictEqual(dropped, ["home"]);
  assert.strictEqual(q.size(), 2);
});

test("sent failure keeps the queue", () => {
  const q = create({ ttlMs: 1000, now: () => 0 });
  const sent = [];
  q.enqueue({ t: "home" }, () => {});
  assert.strictEqual(q.flush((obj) => { sent.push(obj.t); return false; }, () => {}), false);
  assert.strictEqual(q.size(), 1);
  assert.deepStrictEqual(sent, ["home"]);
  assert.strictEqual(q.flush((obj) => { sent.push(obj.t); return true; }, () => {}), true);
  assert.deepStrictEqual(sent, ["home", "home"]);
});

test("sweep removes expired commands", () => {
  let t = 0;
  const q = create({ ttlMs: 1000, now: () => t });
  const dropped = [];
  q.enqueue({ t: "home" }, (obj, reason) => dropped.push(obj.t));
  t = 1001;
  q.sweep((obj, reason) => dropped.push(obj.t));
  assert.deepStrictEqual(dropped, ["home"]);
  assert.strictEqual(q.size(), 0);
});

if (process.exitCode) {
  console.error("cmd queue tests failed");
} else {
  console.log("cmd queue tests passed");
}
