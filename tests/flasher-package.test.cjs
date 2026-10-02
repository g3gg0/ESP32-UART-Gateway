const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const assert = require('node:assert/strict');
const { test } = require('node:test');
const config = JSON.parse(fs.readFileSync('web/flasher/package.json'));

test('flasher scripts compile together and static output has no remote dependencies', () => {
    const sources = config.scripts.map(file => fs.readFileSync(file, 'utf8'));
    new vm.Script(sources.join('\n'));
    const page = fs.readFileSync(config.staticOutput, 'utf8');
    assert.equal(/<script\b[^>]*src=|<link\b[^>]*rel="stylesheet"/.test(page), false);
    assert.equal(page.includes('{{view:'), false);
    for (const match of page.matchAll(/<script>([\s\S]*?)<\/script>/g)) new vm.Script(match[1]);
    for (const id of ['flashTool', 'configTool']) assert.equal(page.split(`id="${id}"`).length - 1, 1);
});

test('embedded firmware and offsets match the build manifest exactly', () => {
    const context = vm.createContext({ Uint8Array, atob, window: {}, navigator: {} });
    for (const file of ['firmware', 'flash', 'config', 'app']) {
        vm.runInContext(fs.readFileSync(`web/flasher/js/${file}.js`, 'utf8'), context);
    }
    const manifest = JSON.parse(fs.readFileSync('build/flasher_args.json'));
    const embedded = vm.runInContext('EMBEDDED_FLASH_CONFIG.files', context);
    assert.equal(embedded.length, Object.keys(manifest.flash_files).length);
    for (const [offset, filename] of Object.entries(manifest.flash_files)) {
        const name = path.basename(filename);
        assert.ok(embedded.some(file => file.offset === offset && file.file === name));
        const data = vm.runInContext(`getEmbeddedBinary(${JSON.stringify(name)})`, context);
        assert.deepEqual(Buffer.from(data), fs.readFileSync(path.join('build', filename)));
    }
    assert.equal(vm.runInContext('isWebSerialSupported()', context), false);
    assert.equal(vm.runInContext('typeof hexdump + ":" + typeof flashHexdump', context), 'function:function');
});
