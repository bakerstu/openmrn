// These are additional test vectors that use different settings for L and IV
// size.
if (index == 50)
{
    Key = hex2str("404142434445464748494a4b4c4d4e4f");
    Nonce = hex2str("10111213141516");
    Adata = hex2str("0001020304050607");
    Payload = hex2str("20212223");
    CT = hex2str("7162015b4dac255d");
    return;
}
if (index == 51) {
    Key = hex2str("C0 C1 C2 C3  C4 C5 C6 C7  C8 C9 CA CB  CC CD CE CF");
    Nonce = hex2str("00 00 00 03  02 01 00 A0  A1 A2 A3 A4  A5");
    Adata = hex2str("00 01 02 03  04 05 06 07");
    Payload = hex2str("08 09 0A 0B  0C 0D 0E 0F  10 11 12 13  14 15 16 17  18 19 1A 1B  1C 1D 1E");
    CT = hex2str("58 8C 97 9A  61 C6 63 D2 F0 66 D0 C2  C0 F9 89 80  6D 5F 6B 61  DA C3 84 17 E8 D1 2C FD  F9 26 E0");
    return;
}

Key = hex2str("26511fb51fcfa75cb4b44da75a6e5a0eb8d9c8f3b906f886df3ba3e6da3a1389");
Nonce = hex2str("72a60f345a1978fb40f28a2fa4");
if (index == 60) {
    Adata = hex2str("");
    Payload = hex2str("30d56ff2a25b83fee791110fcaea48e41db7c7f098a81000");
    CT = hex2str("55f068c0bbba8b598013dd1841fd740fda2902322148ab5e935753e601b79db4ae730b6ae3500731");
    return;
}
if (index == 61) {
    Adata = hex2str("");
    Payload = hex2str("e44b4307234281209bd41f89dbe2cc3fbf68e14df2f7fce4");
    CT = hex2str("816e44353aa38987fc56d39e50f5f0d478f6248f4b1747ba003abc6a4b020625adc8b6cd7bafbd42");
    return;
}
if (index == 62) {
    Adata = hex2str("");
    Payload = hex2str("8db7a73856bcb4007346bb3e00096f69e75e97c0bb960f3b");
    CT = hex2str("e892a00a4f5dbca714c477298b1e538220c052020276b465e7cfa7a208a8b3e6b6377236045df17d");
    return;
}
if (index == 63) {
    Adata = hex2str("");
    Payload = hex2str("48f3ceda4fd390a7eb38f7f5bcd14310af6b5a557e676d44");
    CT = hex2str("2dd6c9e8563298008cba3be237c67ffb68f59f97c787d61a81b39a0c55822e32042b4f8981021090");
    return;
}

Key = hex2str("d24a3d3dde8c84830280cb87abad0bb3");
Nonce = hex2str("f1100035bb24a8d26004e0e24b");
if (index == 64)
{
    Adata = "";
    Payload = hex2str("7c86135ed9c2a515aaae0e9a208133897269220f30870006");
    CT = hex2str("1faeb0ee2ca2cd52f0aa3966578344f24e69b742c4ab37ab112330121"
                 "9c70599b7c373ad4b3ad67b");
    return;
}
if (index == 65)
{
    Adata = "";
    Payload = hex2str("48df73208cdc63d716752df7794807b1b2a80794a2433455");
    CT = hex2str("2bf7d09079bc0b904c711a0b0e4a70ca8ea892d9566f03f8b77a14081"
                 "9f39ef045103e785e1df8c2");
    return;
}
if (index == 66)
{
    Adata = "";
    Payload = hex2str("b99de8168e8c13ea4aef66bdb93133dff5d57e9837ff6ccb");
    CT = hex2str("dab54ba67bec7bad10eb5141ce3344a4c9d5ebd5c3d35b664b0109884"
                 "2a618390619b86e00850b2e");
    return;
}

Key = hex2str("08b0da255d2083808a1b4d367090bacc");
Nonce = hex2str("777828b13679a9e2ca89568233");
if (index == 10) {
    Adata = hex2str("dd");
    Payload = hex2str("1b156d7e2bf7c9a25ad91cff7b0b02161cb78ff9162286b0");
    CT = hex2str("e8b80af4960d5417c15726406e345c5c46831192b03432eed16b6282283e16602331bcca9d51ce76");
    return;
}
if (index == 11) {
    Adata = hex2str("c5");
    Payload = hex2str("032fee9dbffccc751e6a1ee6d07bb218b3a7ec6bf5740ead");
    CT = hex2str("f0828917020651c085e42459c544ec52e99372005362baf308ebeed45f67ef8733737c9c6f82daad");
    return;
}
if (index == 12) {
    Adata = hex2str("68");
    Payload = hex2str("9c4cd65b92070bc382fd18146611defb4204acddfdf6b276");
    CT = hex2str("6fe1b1d12ffd9676197322ab732e80b1183032b65be00628f9b477e3a23bfdfdb619c7bc531fbcce");
    return;
}

Key = hex2str("c0425ed20cd28fda67a2bcc0ab342a49");
if (index == 13) {
    Nonce = hex2str("37667f334dce90");
    Adata = hex2str("0b3e8d9785c74c8f41ea257d4d87495ffbbb335542b12e0d62bb177ec7a164d9");
    Payload = hex2str("4f065a23eeca6b18d118e1de4d7e5ca1a7c0e556d786d407");
    CT = hex2str("768fccdf4898bca099e33c3d40565497dec22dd6e33dcf4384d71be8565c21a455db45816da8158c");
    return;
}
if (index == 14) {
    Nonce = hex2str("f7a5098b2a4d92");
    Adata = hex2str("bc498326755503ff25d02805eb3517221b54eb4fd79af0fcdf9312b2a9ad95f7");
    Payload = hex2str("3e2144e2a381b718962a77e167778bf579957a8fae29612c");
    CT = hex2str("98ce91033fabaa8fe853d347be6cbe5de102fdccf042e7be697b41c9a69acaf8386140ee6e36f406");
    return;
}

Key = hex2str("e6ab9e70a4fb51b01c2e262233e64c0d");
if (index == 15) {
    Nonce = hex2str("74e689eb5af9441dd690a6");
    Adata = hex2str("42f6518ee0fbe42f28e13b4bb2eb60517b37c9744394d9143393a879c3e107c7");
    Payload = hex2str("ba15916733550d7aa82b2f6b117cd3f54c83ddc16cd0288a");
    CT = hex2str("dcc151443288f35d39ed8fae6f0ce1d1eb656f4f7fd65c0b16f322ce85d7c54e71ac560fd4da9651");
    return;
}
if (index == 16) {
    Nonce = hex2str("eb118fb41284bfcb1bc338");
    Adata = hex2str("b5a6067fbac46578cfc8d3fe04108588c9de077eb009249374f205553bba9d02");
    Payload = hex2str("863da00c7accf45418d47c1eda72338734dcc49cd599f328");
    CT = hex2str("d64de7a56146b971e21bf5784d67bab32dd837cfb81591da4a0177883346dc896eb39e8a32bc1393");
    return;
}
Key = hex2str("ac87fef3b76e725d66d905625a387e82");
if (index == 17) {
    Nonce = hex2str("61bf06b9fa5a450d094f3ddcb5");
    Adata = hex2str("0245484bcd987787fe97fda6c8ffb6e7058d7b8f7064f27514afaac4048767fd");
    Payload = hex2str("959403e0771c21a416bd03f3898390e90d0a0899f69f9552");
    CT = hex2str("cabf8aa613d5357aa3e70173d43f1f202b628a61d18e8b572eb66bb8213a515aa61e5f0945cd57f4");
    return;
}
if (index == 18) {
    Nonce = hex2str("2a27257bfaadf23a87df082c57");
    Adata = hex2str("0001dc666c9daf3560daeaf514270db0b5075d295068e6caf231c1de0e1a9300");
    Payload = hex2str("6cbbfa6d736fbcc4cf73ab4d7be537420e0e574ee1f2d1b5");
    CT = hex2str("72d525e6bb312bf2c20b91f41108779789c25720797ebffa4cd9d735f51430275387c565cf1a69bc");
    return;
}
