const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const { test } = require('node:test');
const config = JSON.parse(fs.readFileSync('web/cc3200/package.json'));

test('CC3200 scripts compile together and static output has all eight templates', () => {
    const context = vm.createContext({ window: {}, TextDecoder, Uint8Array, setTimeout, clearTimeout });
    const scripts = config.scripts.map(file => fs.readFileSync(file, 'utf8'));
    new vm.Script(scripts.join('\n'));
    for (const script of scripts) vm.runInContext(script, context);
    const commands = vm.runInContext('CC_COMMANDS', context);
    assert.equal(commands.length, 8);
    const page = fs.readFileSync(config.staticOutput, 'utf8');
    assert.equal(/<script\b[^>]*src=|<link\b[^>]*rel="stylesheet"/.test(page), false);
    assert.equal(/\{\{(?:view:|styles|scripts)/.test(page), false);
    for (const match of page.matchAll(/<script>([\s\S]*?)<\/script>/g)) new vm.Script(match[1]);
    for (const command of commands) {
        assert.equal(page.split(`id="cc-view-${command.id}"`).length - 1, 1);
        let cloned = false;
        const label = {};
        context.document = { getElementById: id => {
            assert.equal(id, `cc-view-${command.id}`);
            return { content: { cloneNode: deep => { assert.equal(deep, true); return command.id; } } };
        } };
        command.render({ replaceChildren: value => { assert.equal(value, command.id); cloned = true; },
            querySelectorAll: () => [label] });
        assert.equal(cloned, true);
        assert.equal(label.textContent, command.description);
    }
});

test('extracted assembler still handles fragmented packets and checksum errors', () => {
    const context = vm.createContext({ Uint8Array });
    vm.runInContext(fs.readFileSync('web/cc3200/js/CC3200PacketAssembler.js', 'utf8'), context);
    const assembler = vm.runInContext('new CC3200PacketAssembler()', context);
    assert.equal(assembler.push(new Uint8Array([0, 4])), null);
    assert.deepEqual(Array.from(assembler.push(new Uint8Array([3, 1, 2]))), [1, 2]);
    assert.throws(() => assembler.push(new Uint8Array([0, 4, 0, 1, 2])), /csum failed/);
});
