"use strict";
(() => {
  const $ = id => document.getElementById(id);
  let token = "";
  try { token = sessionStorage.getItem("warm-wheels-token") || ""; } catch (_) {}
  const fragment = new URLSearchParams(location.hash.slice(1)).get("token");
  if (fragment) { token = fragment; history.replaceState(null,"",location.pathname); }
  $("local-token").value = token;
  async function api(path, body) {
    token = $("local-token").value.trim();
    try { sessionStorage.setItem("warm-wheels-token", token); } catch (_) {}
    const response = await fetch(path, {method:body===undefined?"GET":"POST",headers:{Authorization:`Bearer ${token}`,"Content-Type":"application/json"},body:body===undefined?undefined:JSON.stringify(body),cache:"no-store",signal:AbortSignal.timeout(6500)});
    const data = await response.json();
    if (!response.ok) {
      if(response.status===401) $("local-auth").open=true;
      throw new Error(typeof data.detail==="string"?data.detail:"Check the address and both access tokens.");
    }
    return data;
  }
  function show(data) {
    $("open-stream").hidden = $("disconnect").hidden = !data.connected;
    $("join-status").textContent = data.connected ? `Connected to ${data.address} · ${data.mode === "hardware" ? "Real hardware stream" : "Remote synthetic demo"}. Motors remain under the rover's safety gates.` : "No Raspberry Pi selected.";
    if(data.address) $("pi-address").value=data.address;
  }
  $("join-form").addEventListener("submit",async event=>{
    event.preventDefault(); $("join").disabled=true; $("join-status").textContent="Checking the Pi address, token and camera service…";
    try { show(await api("/api/pi/connect",{address:$("pi-address").value.trim(),token:$("pi-token").value.trim()})); $("pi-token").value=""; }
    catch(error) { $("join-status").textContent=error.name==="TimeoutError"?"Connection timed out. Check Wi-Fi and the Pi service.":error.message; }
    finally { $("join").disabled=false; }
  });
  $("disconnect").addEventListener("click",async()=>{try{show(await api("/api/pi/disconnect",{}));}catch(error){$("join-status").textContent=error.message;}});
  if(token) api("/api/pi/link").then(show).catch(error=>{$("join-status").textContent=error.message;});
})();
