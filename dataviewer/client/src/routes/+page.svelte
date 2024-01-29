<script>
  import { onMount } from "svelte";
  import Modal from "$lib/Modal.svelte";
  import { faTrashCan } from "@fortawesome/free-regular-svg-icons";
  import Fa from "svelte-fa";
  const server = "http://localhost:5000";

  let files = [];
  let dbfile = "";
  let tables = [];
  let tablename = "";
  let tabledata = [];
  let columns = [];
  let columns_toggle = {};

  let savedTempValue = "";
  let modalActive = false;
  let modalText = "";

  // TODO: HARDCODED VALUES
  let datatypes = {
    should_export_as_rlds: "boolean",
    _wrist_image_sample: "image",
  };

  async function updateFiles() {
    const res = await fetch(`${server}/db`);
    files = await res.json();
    if (files) {
      dbfile = files[0];
      updateTables();
    }
  }

  async function updateTables() {
    const res = await fetch(`${server}/db/${dbfile}`);
    tables = await res.json();
    if (tables) {
      tablename = tables[0];
      updateData();
    }
  }

  async function updateData() {
    const res = await fetch(`${server}/db/${dbfile}/${tablename}`);
    tabledata = await res.json();
    columns = tabledata.length > 0 ? Object.keys(tabledata[0]) : [];
    columns_toggle = Object.fromEntries(columns.map((c) => [c, true]));
  }

  function debugPrint() {
    console.log("data", tabledata);
  }

  function saveValue(event) {
    savedTempValue = event.target.value;
    console.log(savedTempValue);
  }

  function confirmChange(event) {
    if (savedTempValue != event.target.value) {
      modalText = `Change value from ${savedTempValue} to ${event.target.value}?`;
      modalActive = true;
    }
  }

  function deleteRow(index) {
    return (event) => {
      modalText = `Delete row ${index}?`;
      modalActive = true;
    }
  }

  function onModalCancel(event) {
    modalActive = false;
  }

  function onModalSave() {
    console.log("save");
    modalActive = false;
  }

  onMount(async function () {
    await updateFiles();
  });
</script>

<style>
  .edit {
    visibility: hidden;
  }
  tr:hover td .edit {
    visibility: visible;
  }
</style>

<Modal
  onConfirm={onModalSave}
  onCancel={onModalCancel}
  active={modalActive}
  text={modalText}
/>

<div class="columns mt-5">
  <div class="column">
    <h1>FogROS RT-X Database Viewer</h1>
  </div>
  <div class="column">
    <button class="button is-pulled-right">Export to RLDS</button>
  </div>
</div>

<div class="columns">
  <div class="column is-narrow">
    <div class="field is-horizontal">
      <label class="field-label is-normal">Database</label>
      <div class="field-body">
        <div class="control">
          <div class="select">
            <select bind:value={dbfile} on:change={updateTables}>
              {#each files as filename}
                <option>{filename}</option>
              {/each}
            </select>
          </div>
        </div>
      </div>
    </div>

    <div class="field is-horizontal">
      <label class="field-label is-normal">Table</label>
      <div class="field-body">
        <div class="control">
          <div class="select">
            <select bind:value={tablename} on:change={updateData}>
              {#each tables as t}
                <option>{t}</option>
              {/each}
            </select>
          </div>
        </div>
      </div>
    </div>

    <div class="field is-horizontal">
      <label class="field-label is-bold">Columns</label>
      <div class="field-body">
        <div class="control">
          {#each columns as col}
            <label class="checkbox">
              <input type="checkbox" bind:checked={columns_toggle[col]} />
              {col}
            </label>
            <br />
          {/each}
        </div>
      </div>
    </div>
  </div>
  <div class="column table-container">
    {#if tabledata.length > 0}
      <table class="table is-striped is-narrow is-hoverable">
        <thead>
          <tr>
            <th></th>
            <!-- for deleting rows -->
            {#each columns as col}
              {#if columns_toggle[col]}
                <th>{col}</th>
              {/if}
            {/each}
          </tr>
        </thead>
        <tbody>
          {#each tabledata.entries() as [i, row]}
            <tr>
              <td>
                <div class="edit" id={"row-"+i} on:click={deleteRow(i)} >
                <Fa
                  icon={faTrashCan}
                  color="red"
                  style="cursor:pointer"
                />
                </div>
              </td>
              {#each Object.entries(row) as [col, val]}
                {#if columns_toggle[col]}
                  <td>
                    {#if col in datatypes}
                      {#if datatypes[col] == "boolean"}
                        <label class="checkbox">
                          <input
                            type="checkbox"
                            bind:checked={tabledata[i][col]}
                            on:change={debugPrint}
                          />
                        </label>
                      {:else if datatypes[col] == "image"}
                        <img src="/dexnet.png" alt="An alt text" />
                      {/if}
                    {:else}
                      <textarea
                        style="border: none; background: transparent; width: 100%; resize:none"
                        on:focusin={saveValue}
                        on:focusout={confirmChange}
                        bind:value={tabledata[i][col]}
                        on:change={debugPrint}
                      />
                    {/if}
                  </td>
                {/if}
              {/each}
            </tr>
          {/each}
        </tbody>
      </table>
    {:else}
      <p>Could not load data. Check your db file?</p>
    {/if}
  </div>
</div>
