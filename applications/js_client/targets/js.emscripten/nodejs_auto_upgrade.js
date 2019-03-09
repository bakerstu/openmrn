/// This example will listen (and advertise on mdns) an LCC connection. Any
/// nodes that connect will be tested for being in formware upgrade mode. If
/// yes, and an update file is available for that node, a firmware upgrade
/// process will be performed on that node.

var lcc;
var mdns = require('mdns');
var port = 12018
var browser;
var fs = require('fs');

/// Dictionary of all currently in-progress firmware upgrades. Used to prevent
/// triggering a second upgrade while a first one is ongoing.
/// key: string version of the node ID.
var pendingUpgrades = {};

/// Test if a node ID is in a given range, and if yes, then downloads a given
/// firmware file to it.
/// @param node_id is a NodeID object for the target node
/// @param id_pat is a regular expression matching on the NodeID.toString()
/// @param file_name is the path to the (binary) download file on the
/// filesystem. If this file does not exist, no download will be performed.
function tryUpgrade(node_id, id_pat, file_name) {
    var s = node_id.toString();
    if (!s.match(id_pat)) {
        return;
    }
    if (s in pendingUpgrades) {
        console.log('node upgrade already pending: ' + s);
        return;
    }
    console.log('Found match for file: ' + file_name);
    if (!fs.existsSync(file_name)) {
        return;
    }
    console.log('Upgrading node: ' + s);
    pendingUpgrades[s] = 1;
    new lcc.BootloaderClient().upgrade(node_id, file_name, true, function(pr) {
        console.log('Progress: ' + pr);
    }, function(err) {
        if (err) {
            console.log('Upgrade for ' + node_id + ' failed: ' + err);
        } else {
            console.log('Upgrade for ' + node_id + ' successful.');
        }
        delete pendingUpgrades[s];
    });
}

/// Checks if we have a firmware for this node. If yes, we run the upgrader.
/// @param node_id is a NodeID object representing the node.
function maybeUpgrade(node_id) {
    tryUpgrade(
        node_id, /^090099dd01[12].*$/, '/tmp/FirmwareUpgrade_LTHR.revb.bin');
}

/// Callback to be invoked when a new node appears on the bus. Will run a PIP
/// request on the node, and if it is in firmware upgrade mode, then launches
/// the downloader.
/// @param node_id is a NodeID object representing the node.
var onNewLccNode = function(node_id) {
    console.log('new node ' + node_id)
    new lcc.PIPClient().lookup(node_id, function(err, data) {
        if (err) {
            console.log("PIP lookup " + node_id + " error " + err); 
        } else {
            console.log("PIP lookup " + node_id + " data " + data);
            if (data.includes('FIRMWARE_UPGRADE_ACTIVE')) {
                maybeUpgrade(node_id);
            }
        }
    });
}

require('./js_client.js')().then(function(Module) {
    lcc = Module
    lcc.startTcpHub(port)
    console.log('starting stack')
    try {
        lcc.startStack();
    } catch (e) {
        if (e === 'SimulateInfiniteLoop') {
            console.log('stack started.');
            lcc.afterStartStack();
        } else {
            console.log('unknown exception');
            throw e;
        }
    }
    browser = new lcc.NodeBrowser(onNewLccNode);
});

var ad = mdns.createAdvertisement(mdns.tcp('openlcb-can'), port, {name: 'openmrn-node-test'} );
ad.start();
