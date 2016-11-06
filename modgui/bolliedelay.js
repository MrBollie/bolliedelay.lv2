function (event) {

    function handle_event (symbol, value) {
        switch (symbol) {
            case 'tempo_out':
                event.icon.find ('[mod-port-symbol=tempo_out]').text(value);
                break;
            default:
                break;
        }
    }

    if (event.type == 'start') {
        var ports = event.ports;
        for (var p in ports) {
            handle_event (ports[p].symbol, ports[p].value);
        }
    }
    else if (event.type == 'change') {
        handle_event (event.symbol, event.value);
    }
}
