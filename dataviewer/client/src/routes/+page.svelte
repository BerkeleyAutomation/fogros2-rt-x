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
  let updateModalActive = false;
  let deleteModalActive = false;
  let exportModalActive = false;
  let modalText = "";
  let exportOptions = {};

  let deleteLoading = false;
  let deleteIndex;

  let message = "";

  // TODO: HARDCODED VALUES
  let datatypes = {
    should_export_as_rlds: "boolean",
    _wrist_image_sample: "image",
    _image_sample: "gif",
    start_time: "time",
    end_time: "time",
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
  }

  function confirmChange(event) {
    if (savedTempValue != event.target.value) {
      modalText = `Change value from ${savedTempValue} to ${event.target.value}?`;
      updateModalActive = true;
    }
  }

  async function exportData(event) {
    modalText = `Export data`;
    const res = await fetch(`${server}/export`);
    exportOptions = await res.json();
    exportModalActive = true;
  }

  async function deleteRow() {
    let id = tabledata[deleteIndex]["id"];
    const res = await fetch(`${server}/db/${dbfile}/${tablename}/delete/${id}`);
    if (res.ok) {
      message = `Deleted row ${id}`;
      tabledata.splice(deleteIndex, 1);
      deleteLoading = false;
      deleteModalActive = false;
    } else {
      message = `Error deleting row ${id}`;
      modalText = `Error deleting row ${id}`;
      deleteLoading = false;
    }
  }

  function showModal(name, data = null) {
    return () => {
      if (name == "deleteRow") {
        deleteModalActive = true;
        deleteIndex = data;
        modalText = `Delete row ${tabledata[deleteIndex]["id"]}?`;
      }
    };
  }

  function onModalCancel(event) {
    updateModalActive = false;
    deleteModalActive = false;
    exportModalActive = false;
  }

  function selectAllColumns(value) {
    return () => {
      columns.forEach((col) => {
        columns_toggle[col] = value;
      });
    };
  }

  function onModalSave() {
    console.log("save");
    updateModalActive = false;
  }

  onMount(async function () {
    await updateFiles();
  });
</script>

<Modal active={updateModalActive}>
  <p>{modalText}</p>
  <button class="button is-success" on:click={confirmChange}>Save</button>
  <button class="button" on:click={onModalCancel}>Cancel</button>
</Modal>

<Modal active={deleteModalActive}>
  <p>{modalText}</p>
  <button
    class={"button is-danger" + (deleteLoading ? " is-loading" : "")}
    on:click={deleteRow}>Delete</button
  >
  <button class="button" on:click={onModalCancel}>Cancel</button>
</Modal>

<Modal active={exportModalActive}>
  <h3>Export to RLDS</h3>

  {#each Object.keys(exportOptions) as option}
    <div class="field is-horizontal">
      <label class="field-label is-normal">{option}</label>
      <div class="field-body">
        <div class="control is-expanded">
          <input
            class="input is-fullwidth"
            type="text"
            bind:value={exportOptions[option]}
          />
          <!-- <div class="select">
            <select>
              {#each values as item}
                <option>{item}</option>
              {/each}
            </select>
          </div> -->
        </div>
      </div>
    </div>
  {/each}

  <div class="field is-horizontal">
    <label class="field-label is-normal">Destination</label>
    <div class="field-body">
      <div class="field">
        <div class="control">
          <div class="select">
            <select>
              <option>Upload to Google</option>
            </select>
          </div>
        </div>
      </div>
    </div>
  </div>

  <button class="button is-info" on:click={confirmChange}>Export</button>
  <button class="button" on:click={onModalCancel}>Cancel</button>
</Modal>

<div class="columns mt-5">
  <div class="column is-narrow">
    <h1>FogROS RT-X Dataset Viewer</h1>
  </div>
  <div class="column">
    <p>{message}</p>
  </div>
  <div class="column is-narrow">
    <button class="button is-pulled-right" on:click={exportData}
      >Export to RLDS</button
    >
  </div>
</div>

<div class="columns">
  <div class="column is-narrow">
    <div class="field is-horizontal">
      <label class="field-label is-normal">File</label>
      <div class="field-body">
        <div class="field">
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
    </div>

    <div class="field is-horizontal">
      <label class="field-label is-normal">Dataset</label>
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
      <label class="field-label is-bold">Features</label>
      <div class="field-body">
        <div class="field">
          <button class="button is-small" on:click={selectAllColumns(true)}
            >All</button
          >
          <button class="button is-small" on:click={selectAllColumns(false)}
            >None</button
          >
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
                <div
                  class="edit"
                  id={"row-" + i}
                  on:click={showModal("deleteRow", i)}
                >
                  <Fa icon={faTrashCan} color="red" style="cursor:pointer" />
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
                      {:else if datatypes[col] == "time"}
                        <!-- value={tabledata[i][col]} -->
                        <textarea
                          style="border: none; background: transparent; width: 100%; resize:none"
                          on:focusin={saveValue}
                          on:focusout={confirmChange}
                          value={new Date(
                            parseInt(tabledata[i][col]) / 1000000
                          ).toISOString()}
                          on:change={debugPrint}
                        />
                      {:else if datatypes[col] == "image"}
                        <img
                          src={server + "/static/image/dexnet.png"}
                          alt="An alt text"
                        />
                      {:else if datatypes[col] == "gif"}
                        <img
                          src={server + "/static/image/test-fogros.gif"}
                          alt="An alt text"
                        />
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

<style>
  .edit {
    visibility: hidden;
  }
  tr:hover td .edit {
    visibility: visible;
  }
</style>
