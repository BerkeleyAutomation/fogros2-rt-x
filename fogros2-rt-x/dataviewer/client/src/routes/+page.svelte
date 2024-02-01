<script>
  import { browser } from "$app/environment";
  import { faTrashCan } from "@fortawesome/free-regular-svg-icons";
  import Fa from "svelte-fa";
  import Modal from "$lib/Modal.svelte";
  import Tags from "svelte-tags-input";

  const server = "http://localhost:5000";

  let files = [];
  let dbfile = "";
  let tables = [];
  let tablename = "";
  let tabledata = [];
  let columns = [];
  let activeCols = {};

  let totalRows = 0;
  let page = 0;
  let rowsPerPage = 20;

  let colSortBy = "";
  let sortDec = false;

  let savedTempValue = "";
  let activeModal = "";
  let modalText = "";
  let confirmEdit = true;

  let orchestrator = "";
  let exportDest = "google";
  let exportLoading = false;

  let exportOptions = {
    orchestrators: [],
    topics: {
      action_topics: [],
      step_topics: [],
      observation_topics: [],
    },
  };

  let buttonLoading = false;
  let deleteIndex = null;
  let updateValueData = null;

  let message = "";

  // TODO: HARDCODED VALUES
  let datatypes = {
    should_export_as_rlds: "boolean",
    _wrist_image_sample: "image",
    _image_sample: "gif",
    start_time: "time",
    end_time: "time",
    id: "id",
  };

  async function fetchFiles() {
    const res = await fetch(`${server}/db`);
    files = await res.json();
    if (res.ok && files) {
      dbfile = files[0];
      return await fetchTables();
    }
  }

  async function fetchTables() {
    const res = await fetch(`${server}/db/${dbfile}`);
    tables = await res.json();
    if (tables) {
      tablename = tables[0];
      return await fetchData();
    }
  }

  async function fetchData() {
    message = "Fetching data...";
    const res = await fetch(
      `${server}/db/${dbfile}/${tablename}/${rowsPerPage}/${page}?sortby=${
        sortDec ? "-" : ""
      }${colSortBy}`
    );
    const result = await res.json();
    tabledata = result["data"];
    totalRows = result["count"];

    const oldColumns = columns;
    columns = tabledata.length > 0 ? Object.keys(tabledata[0]) : [];
    if (
      !(oldColumns.length > 0 && columns.every((c, i) => c === oldColumns[i]))
    ) {
      activeCols = Object.fromEntries(columns.map((c) => [c, true]));
    }

    message = "";
    return true;
  }

  function saveValue(event) {
    savedTempValue = event.target.value;
  }

  async function updateValue() {
    try {
      const res = await fetch(
        `${server}/db/${dbfile}/${tablename}/update/${updateValueData["id"]}?col=${updateValueData["col"]}&val=${updateValueData["value"]}`
      );
      message = `Updated ${updateValueData["col"]} to ${updateValueData["value"]}`;
      activeModal = "";
    } catch {
      updateValueData["target"].value = savedTempValue;
      message = `Error updating value`;
      modalText = `Error updating value`;
      buttonLoading = false;
    }
  }

  function updateExportOption(option) {
    return (value) => {
      exportOptions[option] = value;
    };
  }

  async function exportData(event) {
    buttonLoading = true;
    try {
      const res = await fetch(
        `${server}/export/${dbfile}/${exportDest}?orchestrator=${orchestrator}` +
          `&o=${exportOptions["topics"]["observation_topics"]}` +
          `&s=${exportOptions["topics"]["step_topics"]}` +
          `&a=${exportOptions["topics"]["action_topics"]}`
      );
      if (res.ok) {
        message = `Exported data to ${exportData}`;
        activeModal = "";
      } else {
        throw new Error(`Something went wrong: ${res.status}`);
      }
    } catch (error) {
      message = `Error exporting data`;
      modalText = `Error, check console for details.`;
      console.log(error)
    } finally {
      buttonLoading = false;
    }
  }

  async function deleteRow() {
    let id = tabledata[deleteIndex]["id"];
    const res = await fetch(`${server}/db/${dbfile}/${tablename}/delete/${id}`);
    if (res.ok) {
      await fetchData();
      message = `Deleted row ${id}`;
      buttonLoading = false;
      activeModal = "";
    } else {
      message = `Error deleting row ${id}`;
      modalText = `Error deleting row ${id}`;
      buttonLoading = false;
    }
  }

  function showModal(name, data = null) {
    return async (event) => {
      if (name == "delete") {
        if (confirmEdit) {
          activeModal = "delete";
          deleteIndex = data;
          modalText = `Delete row ${tabledata[deleteIndex]["id"]}?`;
        } else {
          await deleteRow();
        }
      } else if (name == "update") {
        if (savedTempValue != event.target.value) {
          if (confirmEdit) {
            activeModal = "update";
            modalText = `Change value from ${savedTempValue} to ${event.target.value}?`;
            updateValueData = data;
            updateValueData["id"] = tabledata[updateValueData["i"]]["id"];
            updateValueData["value"] = event.target.value;
            updateValueData["target"] = event.target;
          } else {
            await updateValue();
          }
        }
      } else if (name == "export") {
        activeModal = "export";
        modalText = "";
        const res = await fetch(`${server}/export/options`);
        exportOptions = await res.json();
        if (exportOptions["orchestrators"].length > 0) {
          orchestrator = exportOptions["orchestrators"][0];
        }
      }
    };
  }

  function onModalCancel() {
    activeModal = "";
  }

  function selectAllColumns(value) {
    return () => {
      columns.forEach((col) => {
        activeCols[col] = value;
      });
    };
  }

  function changePage(p, force = false) {
    return () => {
      p = Math.min(Math.max(0, p), (totalRows / rowsPerPage) >> 0);
      if (force || page != p) {
        page = p;
        fetchData();
      }
    };
  }

  function sortBy(col) {
    return async () => {
      sortDec = colSortBy == col ? !sortDec : false;
      colSortBy = col;
      await fetchData();
    };
  }
</script>

<!-- Modals -->
<Modal active={activeModal == "update"}>
  <p>{modalText}</p>
  <button class="button is-success" on:click={updateValue}>Save</button>
  <button class="button" on:click={onModalCancel}>Cancel</button>
</Modal>

<Modal active={activeModal == "delete"}>
  <p>{modalText}</p>
  <button
    class={"button is-danger" + (buttonLoading ? " is-loading" : "")}
    on:click={deleteRow}>Delete</button
  >
  <button class="button" on:click={onModalCancel}>Cancel</button>
</Modal>

<Modal active={activeModal == "export"}>
  <h3>Export to RLDS</h3>

  <div class="field is-horizontal">
    <label class="field-label is-normal">Orchestrator</label>
    <div class="field-body">
      <div class="field">
        <div class="control">
          <div class="select">
            <select bind:value={orchestrator}>
              {#each exportOptions["orchestrators"] as o}
                <option>{o}</option>
              {/each}
            </select>
          </div>
        </div>
      </div>
    </div>
  </div>

  {#each Object.keys(exportOptions["topics"]) as option}
    <div class="field is-horizontal">
      <label class="field-label is-normal">{option}</label>
      <div class="field-body">
        <div class="field is-expanded">
          <!-- check keycodes https://www.toptal.com/developers/keycode -->
          <!-- bind:tags={exportOptions["topics"][option]} -->
          <Tags
            bind:tags={exportOptions["topics"][option]}
            on:tags={updateExportOption(option)}
            addKeys={[9, 32, 188, 13]}
            allowPaste={true}
            allowDrop={true}
            onlyUnique={true}
            placeholder={""}
            allowBlur={true}
          />
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
            <select bind:value={exportDest}>
              <option value="google">Upload to Google</option>
            </select>
          </div>
        </div>
      </div>
    </div>
  </div>
  <button
    class={"button is-info" + (buttonLoading ? " is-loading" : "")}
    on:click={exportData}>Export</button
  >
  <button class="button" on:click={onModalCancel}>Cancel</button>
  <div class="is-inline-block p-2">
    {modalText}
  </div>
</Modal>
<!-- End Modals -->

<div class="columns mt-5">
  <div class="column is-narrow">
    <h1>FogROS RT-X Dataset Viewer</h1>
  </div>
  <div class="column mt-3">
    <p>{message}</p>
  </div>
  <div class="column is-narrow">
    <button
      class={"button is-pulled-right" + (exportLoading ? " is-loading" : "")}
      on:click={showModal("export")}>Export to RLDS</button
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
              <select bind:value={dbfile} on:change={fetchTables}>
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
            <select bind:value={tablename} on:change={fetchData}>
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
          <div class="mb-2">
            <button class="button is-small" on:click={selectAllColumns(true)}
              >All</button
            >
            <button class="button is-small" on:click={selectAllColumns(false)}
              >None</button
            >
          </div>
          <div class="control">
            {#each columns as col}
              <label class="checkbox">
                <input type="checkbox" bind:checked={activeCols[col]} />
                {col}
              </label>
              <br />
            {/each}
          </div>
        </div>
      </div>
    </div>

    <div class="field is-horizontal">
      <label class="field-label is-normal">Options</label>
      <div class="field-body">
        <label class="checkbox mt-3">
          <input type="checkbox" bind:checked={confirmEdit} />
          Confirm for edit/delete
        </label>
      </div>
    </div>

    <div class="field is-horizontal">
      <label class="field-label is-normal"></label>
      <div class="field-body">
        <input
          class="input is-small mr-1"
          type="input"
          bind:value={rowsPerPage}
          on:focusout={changePage(page, true)}
          style="width:5em"
        />
        per page
      </div>
    </div>
  </div>

  <div class="column table-container">
    {#await browser ? fetchFiles() : Promise.resolve()}
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
      <progress class="progress is-normal is-light" max="100"></progress>
    {:then done}
      <div class="level">
        <div class="level-left">
          <div class="level-item">
            <button class="button is-small" on:click={changePage(page - 1)}
              >{"<"}</button
            >
            <button class="button is-small" on:click={changePage(page + 1)}
              >{">"}</button
            >
          </div>
          <div class="level-item">
            <p>
              Showing {1 + page * rowsPerPage}-{page * rowsPerPage +
                tabledata.length} of {totalRows} data points
            </p>
          </div>
        </div>
      </div>

      <table class="table is-striped is-narrow is-hoverable">
        <thead>
          <tr>
            <th>
              <!-- extra first column for edittools (delete) -->
            </th>
            {#each columns as col}
              {#if activeCols[col]}
                <th
                  class={"table-column-hover " +
                    (colSortBy == col
                      ? sortDec
                        ? "has-text-danger has-background-danger-light"
                        : "has-text-info has-background-info-light"
                      : "")}
                  style="cursor:pointer"
                  on:click={sortBy(col)}>{col}</th
                >
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
                  on:click={showModal("delete", i)}
                >
                  <Fa icon={faTrashCan} color="red" style="cursor:pointer" />
                </div>
              </td>
              {#each Object.entries(row) as [col, val]}
                {#if activeCols[col]}
                  <td>
                    {#if col in datatypes}
                      {#if datatypes[col] == "boolean"}
                        <label class="checkbox">
                          <input
                            type="checkbox"
                            bind:checked={tabledata[i][col]}
                          />
                        </label>
                      {:else if datatypes[col] == "id"}
                        {val}
                      {:else if datatypes[col] == "time"}
                        <textarea
                          style="border: none; background: transparent; width: 100%; resize:none"
                          readonly
                          value={new Date(
                            parseInt(tabledata[i][col]) / 1000000
                          ).toISOString()}
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
                        on:focusout={showModal("update", { col: col, i: i })}
                        bind:value={tabledata[i][col]}
                      />
                    {/if}
                  </td>
                {/if}
              {/each}
            </tr>
          {/each}
        </tbody>
      </table>
    {:catch error}
      <p>Could not load data. Check your db file?</p>
    {/await}
  </div>
</div>

<style>
  .edit {
    visibility: hidden;
  }
  tr:hover td .edit {
    visibility: visible;
  }
  .table-column-hover:hover {
    background-color: #e8e8e8;
    user-select: none;
  }

  :global(.svelte-tags-input-tag) {
    background-color: #dfdfdf !important;
    color: #000000 !important;
  }
</style>
